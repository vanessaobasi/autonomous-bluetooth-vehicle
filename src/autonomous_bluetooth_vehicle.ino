/*
 * Autonomous Bluetooth Vehicle
 * 
 * This program controls an autonomous car that can operate in two modes: virtual
 * joystick control and line tracking. It integrates multiple features such as motor
 * control, sensor data processing, and EEPROM-based data logging for accelerometer
 * readings.
 *
 * Author: Vanessa Obasi
 * Date: 12/10/2024
 */

#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h> // Include the EEPROM library

const unsigned long BAUD_RATE = 9600;

// Pins and variables for the motors, MOTOR PARAMETERS
//-----------------------------------------------
// MOTOR LEFT PINS
const int ENA = 9;
const int IN1 = 6;
const int IN2 = 5;
// MOTOR RIGHT PINS
const int ENB = 3;
const int IN3 = 4;
const int IN4 = 10;

const float Lspeedfactor = 1;
const float Rspeedfactor = 0.67;
const int MINIMUM_MOTOR_SPEED = 65;
//---------------------------------------------------

// Line track variables and pins
//----------------------------------------------------
int IRvalueA = 0;
int IRvalueB = 0;
int IRvalueR = 0;
int IRvalueL = 0;

#define LL 2
#define RL 13
#define pinIRR A1
#define pinIRL A0

const int MIN_SPEED = 55;
const int MAX_SPEED = 80;
//-----------------------------------------------------

// Variables for mode changes
//-------------------------------------------
const int BUTTON_CAR_MODE = 1;
const int BUTTON_LINE_TRACKER_MODE = 2;
int currentMode = 1; // Default mode: Car Drive Mode
bool isRecording = false;
//---------------------------------------------------

// HM10 BLUETOOTH MODULE PINS
const int BLUETOOTH_TX = 13;
const int BLUETOOTH_RX = 12;

// Variables for recording the acceleration and eeprom
// Global variables to store the maximum values across multiple function calls
float max_ax_g = 0;
float max_ay_g = 0;
//------------------------------------------------------------

MPU6050 mpu;

// EEPROM address to store real-time data
int eepromAddr = 9; // Start storing real-time data from address 9
int startingPosition = 9;
//-------------------------------------------------------------------

SoftwareSerial softSerial(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue phone(softSerial);

// Motor control functions
void motorBrake() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void motorSetForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void motorSetBackward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void setPins() {
  //-----------------
  // For line tracker
  pinMode(RL, INPUT);
  pinMode(pinIRR, INPUT);
  pinMode(LL, INPUT);
  pinMode(pinIRL, INPUT);
  //----------------
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

//----------------
// Line tracker sensor functions
void readLineSensors() {
  IRvalueA = analogRead(pinIRR);
  IRvalueR = digitalRead(RL);
  IRvalueB = analogRead(pinIRL);
  IRvalueL = digitalRead(LL);
}

void followLine() {
  int leftMotorSpeed = MIN_SPEED;
  int rightMotorSpeed = MIN_SPEED;
  motorSetForward();

  // Read the sensor values to determine the robot's position relative to the line.
  readLineSensors(); // Adjust the robot's movement based on sensor readings

  if (IRvalueL == LOW && IRvalueR == HIGH) {
    // Move right
    leftMotorSpeed = MIN_SPEED + 50;  // Adjust to steer right
    rightMotorSpeed = MIN_SPEED - 30; // Adjust to steer right
  } else if (IRvalueL == HIGH && IRvalueR == LOW) {
    // Move left
    leftMotorSpeed = MIN_SPEED - 30;  // Adjust to steer left
    rightMotorSpeed = MIN_SPEED + 40; // Adjust to steer left
  } else if (IRvalueL == IRvalueR) {
    // Move straight
    leftMotorSpeed = MIN_SPEED;
    rightMotorSpeed = MIN_SPEED;
  }

  // Apply the motor speeds
  analogWrite(ENA, leftMotorSpeed);
  analogWrite(ENB, rightMotorSpeed);
}

//------------
void controlDrive() {
  int throttle = phone.getThrottle() - 49;
  int steering = phone.getSteering() - 49;

  if (throttle == 0) {
    motorBrake();
    return;
  }

  int mappedSpeed = map(abs(throttle), 0, 50, MINIMUM_MOTOR_SPEED, 255);
  int reducedSpeed = map(abs(steering), 0, 50, mappedSpeed, MINIMUM_MOTOR_SPEED);
  int leftMotorSpeed = mappedSpeed;
  int rightMotorSpeed = mappedSpeed;

  if (throttle > 0) {
    motorSetForward();
    if (steering > 0) rightMotorSpeed = reducedSpeed;
    else if (steering < 0) leftMotorSpeed = reducedSpeed;
  } else {
    motorSetBackward();
    if (steering > 0) rightMotorSpeed = reducedSpeed;
    else if (steering < 0) leftMotorSpeed = reducedSpeed;
  }

  leftMotorSpeed = leftMotorSpeed * Lspeedfactor;
  rightMotorSpeed = rightMotorSpeed * Rspeedfactor;

  analogWrite(ENA, leftMotorSpeed);
  analogWrite(ENB, rightMotorSpeed);

  // For testing
  /*
  Serial.print("throttle: "); Serial.print(throttle);
  Serial.print("\tsteering: "); Serial.print(steering);
  Serial.print("\tmappedSpeed: "); Serial.print(mappedSpeed);
  Serial.print("\treducedSpeed: "); Serial.print(reducedSpeed);
  Serial.print("\tleftMotorSpeed: "); Serial.print(leftMotorSpeed);
  Serial.print("\trightMotorSpeed: "); Serial.println(rightMotorSpeed);
  */
}

// For the MPU6050 and the EEPROM
void storeRealTimeData() {
  // Variables to store accelerometer data
  int16_t ax, ay, az; // Read raw data from MPU6050
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert accelerometer raw values to 'g'
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;

  // Track and update the maximum X and Y acceleration values
  if (abs(ax_g) > max_ax_g) max_ax_g = ax_g;
  if (abs(ay_g) > max_ay_g) max_ay_g = ay_g;

  // Store the maximum X-Y accelerometer values in EEPROM (positions 0-8)
  EEPROM.put(0, max_ax_g); // Store max_ax_g in EEPROM starting from address 0
  EEPROM.put(4, max_ay_g); // Store max_ay_g in EEPROM starting from address 4

  // Store real-time data (X, Y accelerations) in EEPROM starting from address 9
  EEPROM.put(eepromAddr, ax_g);     // Store ax_g in EEPROM
  EEPROM.put(eepromAddr + 4, ay_g); // Store ay_g in EEPROM

  // Print the stored real-time data to verify
  Serial.print("Real-time Data: ");
  Serial.print("X: "); Serial.print(ax_g);
  Serial.print(" Y: "); Serial.println(ay_g);

  // Increment the EEPROM address for the next data point
  eepromAddr += 8; // Move by 8 bytes (4 bytes for X and 4 bytes for Y)

  // If the EEPROM address exceeds 1023, reset it back to 9
  if (eepromAddr > 1023) {
    eepromAddr = 9; // Start overwriting from address 9 again
  }
}

void readingdata() {
  // Loop through EEPROM starting from the given position
  for (int i = startingPosition; i < EEPROM.length() - 1; i += 8) { // Move by 8 bytes for each pair of X, Y data
    // Declare variables to store X and Y acceleration values
    float xAccel;
    float yAccel;

    // Read X and Y acceleration values (stored as floats)
    EEPROM.get(i, xAccel);     // Read X acceleration value
    EEPROM.get(i + 4, yAccel); // Read Y acceleration value

    // Print the retrieved data
    Serial.print("Reading Data from EEPROM, Group starting at address ");
    Serial.print(i);
    Serial.print(": X = ");
    Serial.print(xAccel);
    Serial.print(", Y = ");
    Serial.println(yAccel);
  }

  // Optionally, read the stored maximum values from EEPROM
  float storedMaxX, storedMaxY;
  EEPROM.get(0, storedMaxX); // Read X acceleration maximum value (stored as float)
  EEPROM.get(4, storedMaxY); // Read Y acceleration maximum value (stored as float)

  // Print the stored maximum values from EEPROM
  Serial.print("Stored maximum X acceleration: ");
  Serial.println(storedMaxX);
  Serial.print("Stored maximum Y acceleration: ");
  Serial.println(storedMaxY);
}

void clearEEPROM() {
  // Loop through all 1024 bytes of the EEPROM and set them to 0xFF
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0xFF); // Write 0xFF to each byte
  }
  Serial.println("EEPROM cleared!");
}

void setup() {
  delay(500);
  softSerial.begin(BAUD_RATE);
  Serial.begin(BAUD_RATE);
  setPins();
  motorBrake();
  Serial.println("Setup Complete. Default: Car Drive Mode");

  // Initialize the MPU6050
  Wire.begin();     // Start the I2C communication
  mpu.initialize(); // Initialize the MPU6050

  // Check if the MPU6050 is connected properly
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1); // Stay here if connection fails
  }
  Serial.println("MPU6050 connection successful!");
}

void loop() {
  int button = phone.getButton();
  int sliderId = phone.getSliderId();
  int sliderVal = phone.getSliderVal();
  String str = phone.getText();

  if (button == 0) {
    currentMode = 0;
  } else if (button == BUTTON_CAR_MODE) {
    currentMode = 1; // Set to Car Drive Mode
    Serial.println("Car Drive Mode Activated");
  } else if (button == BUTTON_LINE_TRACKER_MODE) {
    currentMode = 2; // Set to Line Tracker Mode
    Serial.println("Line Tracker Mode Activated");
  }

  if (currentMode == 0) {
    motorBrake();
  } else if (currentMode == 1) {
    controlDrive(); // Run Car Drive Mode
  } else if (currentMode == 2) {
    followLine(); // Run Line Tracker Mode
  }

  if (sliderId == 3) {
    if (sliderVal > 130) {
      if (!isRecording) { // Start recording only once
        isRecording = true;
        Serial.println("Starting data recording...");
      }
    } else {
      isRecording = false;
    }
  }

  // If the recording flag is true, continue storing real-time data
  if (isRecording) {
    clearEEPROM();
    storeRealTimeData();
  }

  if (str == "r ") {
    readingdata();
  }

  delay(100); // Small delay for stability

  /* testing
  if (sliderId != -1) {
    Serial.print("Slider ID: ");
    Serial.print(sliderId);
    Serial.print("\tValue: ");
    Serial.println(sliderVal);
  }
  if (str != "") {
    Serial.println(str);
  }
  */
}
