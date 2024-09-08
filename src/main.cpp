#include <Arduino.h>
#include <EEPROM.h>
#include <IntervalTimer.h>
#include <ArduinoJson.h>

const int inputPin1 = 15;  // Pin for system 1 input (RPM-based input)
const int inputPin2 = 16;  // Pin for system 2 input (trigger-based input)
const int pulsePin1 = 2;  // Pin to emit pulses for system 1 (output 1)
const int pulsePin2 = 4;  // Pin to emit pulses for system 2 (output 2)

const float smoothingFactor = 0.9;  // Smoothing factor (can be adjusted as needed)

#define MAGIC_NUMBER 0xDEADBEEF
#define MAGIC_ADDR 0
#define TABLE_START_ADDR sizeof(unsigned long)
#define STEP_SIZE 100
#define MAX_RPM 10000
#define JSON_BUFFER_SIZE 512  // Buffer size for parsing JSON input

// Function prototypes
void loadDegreeTableToRAM();
unsigned long getInterpolatedDelay(float rpm);
void onTimer0();
void onTimer1();
void onTimer2();
void onTimer3();

IntervalTimer timer0;  // Timer for initial delay for output 1
IntervalTimer timer1;  // Timer for initial delay for output 2
IntervalTimer timer2;  // Timer for pulse width for output 1
IntervalTimer timer3;  // Timer for pulse width for output 2

volatile unsigned long lastPulse = 0;  // Store the time of the last pulse for RPM calculation
volatile unsigned long timeForOneRev = 0;  // Time for one full revolution
volatile unsigned long delayForRPM = 0;  // Delay for both systems based on RPM
volatile bool timeUpdated1 = false;  // Flag to indicate RPM update
float smoothedRPM = 0;  // Smoothed RPM value

unsigned long delayTableInMicroseconds[MAX_RPM / STEP_SIZE + 1];  // RAM storage for time delays

// Example defaultDegreeTable definition
const float defaultDegreeTable[] = {
  10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,   // 0 to 1000 RPM
  10.5, 10.9, 11.4, 11.8, 12.3, 12.8, 13.2, 13.7, 14.2, 14.6,   // 1100 to 2000 RPM
  15.1, 15.5, 16.0, 16.5, 16.9, 17.4, 17.8, 18.3, 18.8, 19.2,   // 2100 to 3000 RPM
  19.7, 20.2, 20.6, 21.1, 21.5, 22.0, 22.5, 22.9, 23.4, 23.8,   // 3100 to 4000 RPM
  24.3, 24.8, 25.2, 25.7, 26.2, 26.6, 27.1, 27.5, 28.0, 28.0,   // 4100 to 5000 RPM
  28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0    // Above 5000 RPM
};

// Interrupt for system 1 (updates RPM and triggers output 1)
void handleInterrupt1() {
  timer0.begin(onTimer0, delayForRPM);
  unsigned long now = micros();
  timeForOneRev = now - lastPulse;
  lastPulse = now;

  timeUpdated1 = true;  // Set flag to calculate new RPM in the loop
}

// Interrupt for system 2 (triggers output 2 based on RPM from system 1)
void handleInterrupt2() {
  // Set timer for system 2 to fire a pulse after the RPM-based delay
  timer1.begin(onTimer1, delayForRPM);
}

// Timer 0 ISR (handles initial pulse delay for output 1)
void onTimer0() {
  digitalWrite(pulsePin1, HIGH);  // Start the pulse on output 1
  timer2.begin(onTimer2, 10);  // Timer 2 will handle pulse width for output 1 (e.g., 10µs)
}

// Timer 1 ISR (handles initial pulse delay for output 2)
void onTimer1() {
  digitalWrite(pulsePin2, HIGH);  // Start the pulse on output 2
  timer3.begin(onTimer3, 10);  // Timer 3 will handle pulse width for output 2 (e.g., 10µs)
}

// Timer 2 ISR (handles pulse width for output 1)
void onTimer2() {
  digitalWrite(pulsePin1, LOW);  // End the pulse on output 1
}

// Timer 3 ISR (handles pulse width for output 2)
void onTimer3() {
  digitalWrite(pulsePin2, LOW);  // End the pulse on output 2
}



void loadDegreeTableToRAM() {
  int numEntries = (MAX_RPM / STEP_SIZE) + 1;

  for (int i = 0; i < numEntries; i++) {
    float degreeDelay;
    EEPROM.get(TABLE_START_ADDR + i * sizeof(float), degreeDelay);

    if (degreeDelay == 0xFFFFFFFF) {
      degreeDelay = defaultDegreeTable[i];  // Use default if uninitialized
    }

    // Convert delay from degrees to microseconds (stored in delayTableInMicroseconds)
    delayTableInMicroseconds[i] = (degreeDelay * 60e6) / (360 * (i * STEP_SIZE));
  }
}

// Function to get interpolated delay for any RPM
unsigned long getInterpolatedDelay(float rpm) {
  if (rpm < 0 || rpm > MAX_RPM) {
    return 0;
  }

  int index1 = rpm / STEP_SIZE;
  int index2 = index1 + 1;

  unsigned long delay1, delay2;
  delay1 = delayTableInMicroseconds[index1];

  if (index2 * STEP_SIZE <= MAX_RPM) {
    delay2 = delayTableInMicroseconds[index2];
  } else {
    delay2 = delay1;
  }

  int rpm1 = index1 * STEP_SIZE;
  int rpm2 = index2 * STEP_SIZE;

  return delay1 + ((rpm - rpm1) * (delay2 - delay1)) / (rpm2 - rpm1);
}

void handleJsonInput(const char* jsonInput) {
  // Allocate a buffer to hold the JSON document
  JsonDocument doc;

  // Parse the JSON input
  DeserializationError error = deserializeJson(doc, jsonInput);

  // Check if the JSON parsing was successful
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.f_str());
    return;
  }

  // Check the type of update (either "point" or "block")
  const char* type = doc["type"];

  // Handle "point" type: Update a single delay value at a specific index
  if (strcmp(type, "point") == 0) {
    int idx = doc["idx"];
    float value = doc["value"];

    // Check if the index is valid
    int numEntries = (MAX_RPM / STEP_SIZE) + 1;
    if (idx >= 0 && idx < numEntries) {
      EEPROM.put(TABLE_START_ADDR + idx * sizeof(float), value);
      Serial.print("Updated delay for index ");
      Serial.print(idx);
      Serial.print(" to ");
      Serial.println(value);
    } else {
      Serial.println("Invalid index in point update.");
    }

  // Handle "block" type: Update multiple delay values at once
  } else if (strcmp(type, "block") == 0) {
    JsonArray values = doc["values"];

    // Update to use size_t for unsigned comparison
    size_t numEntries = (MAX_RPM / STEP_SIZE) + 1;
    if (values.size() <= numEntries) {
      for (size_t i = 0; i < values.size(); i++) {
        float value = values[i];
        EEPROM.put(TABLE_START_ADDR + i * sizeof(float), value);
      }
      Serial.println("Block update successful.");
    } else {
      Serial.println("Too many values in block update.");
    }

  } else {
    Serial.println("Unknown type in JSON input.");
  }

}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Configure input and output pins
  pinMode(inputPin1, INPUT_PULLUP);
  pinMode(inputPin2, INPUT_PULLUP);
  pinMode(pulsePin1, OUTPUT);
  pinMode(pulsePin2, OUTPUT);
  digitalWrite(pulsePin1, LOW);
  digitalWrite(pulsePin2, LOW);

  // Attach interrupts for input signals
  attachInterrupt(digitalPinToInterrupt(inputPin1), handleInterrupt1, RISING);  // Input 1 for RPM
  attachInterrupt(digitalPinToInterrupt(inputPin2), handleInterrupt2, RISING);  // Input 2 for trigger

  // Initialize EEPROM
  unsigned long storedMagicNumber;
  EEPROM.get(MAGIC_ADDR, storedMagicNumber);

  // EEPROM initialization and writing default data if needed
  if (storedMagicNumber != MAGIC_NUMBER) {
    Serial.println("EEPROM not initialized, writing default data...");
    EEPROM.put(MAGIC_ADDR, MAGIC_NUMBER);

    // Calculate number of entries based on STEP_SIZE and MAX_RPM
    int numEntries = (MAX_RPM / STEP_SIZE) + 1;  // +1 to account for 0 RPM

    for (int i = 0; i < numEntries; i++) {
      EEPROM.put(TABLE_START_ADDR + i * sizeof(float), defaultDegreeTable[i]);  // Use float for degrees
    }

    Serial.println("Default table stored in EEPROM successfully!");
  } else {
    Serial.println("EEPROM already initialized.");
  }

  loadDegreeTableToRAM();
}


void loop() {
  // Handle RPM update for system 1
  if (timeUpdated1) {
    float rpm = 60000000.0 / timeForOneRev;
    // Apply exponential moving average for smoothing
    smoothedRPM = smoothingFactor * smoothedRPM + (1 - smoothingFactor) * rpm;

    // Get the interpolated delay based on the smoothed RPM from EEPROM
    delayForRPM = getInterpolatedDelay(smoothedRPM);

    // Set timer for system 1 to fire a pulse after the RPM-based delay
    timer0.begin(onTimer0, delayForRPM);

    Serial.print("RPM: ");
    Serial.print(rpm);
    Serial.print(" | Delay for RPM: ");
    Serial.println(delayForRPM);

    // Reset the flag
    // Reset the flag
    timeUpdated1 = false;
  }

  // Check for serial input to handle JSON formatted data
  if (Serial.available() > 0) {
    String jsonInput = Serial.readStringUntil('\n');  // Read the JSON string from serial input
    handleJsonInput(jsonInput.c_str());  // Pass the JSON input to the handler function
  }
  
}

