#include <Arduino.h>

// Define GPIO pins for motor control
#define IN1 5   // D5
#define IN2 18  // D18
#define IN3 19  // D19
#define IN4 21  // D21

// Define touch input pins for ESP32
#define TOUCH_OPEN 12     // GPIO12 corresponds to Touch5
#define TOUCH_CLOSE 13    // GPIO13 corresponds to Touch4

// Touch threshold - adjust based on your needs
#define TOUCH_THRESHOLD 50  // ESP32 touch values typically range from 0 to 100

// Step sequence for the 28BYJ-48 motor (half-step sequence for smoother motion)
int stepSequence[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

int stepIndex = 0;
int stepDelay = 1;  // milliseconds between steps

unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 500; // Print debug every 500ms

void stepMotor(bool direction) {
  digitalWrite(IN1, stepSequence[stepIndex][0]);
  digitalWrite(IN2, stepSequence[stepIndex][1]);
  digitalWrite(IN3, stepSequence[stepIndex][2]);
  digitalWrite(IN4, stepSequence[stepIndex][3]);
  
  if (direction) {
    stepIndex = (stepIndex + 1) % 8;  // Forward direction
  } else {
    stepIndex = (stepIndex - 1 + 8) % 8;  // Backward direction
  }
}

void stopMotor() {
  // Turn off all coils
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}

void setup() {
  // Start Serial communication
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Starting continuous touch-controlled claw...");
  
  // Set motor pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize with motor stopped
  stopMotor();
  
  Serial.println("Setup complete. Touch and hold pins 2 and 4 to control the claw.");
  Serial.println("Motor pins initialized on 5, 18, 19, 21");
}

void loop() {
  // Read touch values
  uint16_t touchOpenVal = touchRead(TOUCH_OPEN);
  //uint16_t touchOpenVal = 60;
  uint16_t touchCloseVal = touchRead(TOUCH_CLOSE);
  
  // Print debug information every DEBUG_INTERVAL milliseconds
  unsigned long currentTime = millis();
  if (currentTime - lastDebugTime >= DEBUG_INTERVAL) {
    Serial.printf("Touch Values - Open(GPIO12): %d, Close(GPIO13): %d\n", 
                 touchOpenVal, touchCloseVal);
    lastDebugTime = currentTime;
  }
  if (touchOpenVal < TOUCH_THRESHOLD) {
    // Move in opening direction while touched
    stepMotor(true);
    delay(stepDelay);
    Serial.println("Opening...");
  } 
  else if (touchCloseVal < TOUCH_THRESHOLD) {
    // Move in closing direction while touched
    stepMotor(false);
    delay(stepDelay);
    Serial.println("Closing...");
  }
  else {
    // No touch detected, stop motor
    stopMotor();
  }
  
  // Small delay to prevent excessive loop iterations
  delay(1);
}