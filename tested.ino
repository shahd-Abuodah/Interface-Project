#include <Wire.h>
#include <VL53L0X.h>

// Motor pins
#define IN1 13
#define IN2 12
#define IN3 14
#define IN4 27
#define ENA 26
#define ENB 25

// IR sensor pins
#define IR_LEFT 34
#define IR_RIGHT 35

// LiDAR
VL53L0X lidar;

void setup() {
  // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // IR sensors setup
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // LiDAR setup
  Wire.begin();
  lidar.init();
  lidar.setTimeout(500);

  // Serial setup
  Serial.begin(115200);

  // Seed the random number generator
  randomSeed(analogRead(0));
}

void loop() {
  // Read sensor data
  bool irLeft = digitalRead(IR_LEFT);   // 1 = wall detected, 0 = clear
  bool irRight = digitalRead(IR_RIGHT); // 1 = wall detected, 0 = clear
  int distance = lidar.readRangeSingleMillimeters() / 10; // LiDAR distance in cm

  // Print sensor data for debugging
  Serial.print("Left IR: ");
  Serial.print(irLeft);
  Serial.print(" | Right IR: ");
  Serial.print(irRight);
  Serial.print(" | LiDAR Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Random Mouse Algorithm with IR sensor integration
  if (distance < 15) { // If obstacle is detected in front
    Serial.println("Obstacle detected! Stopping and choosing a new direction...");
    stopMotors(); // Stop the robot
    delay(500); // Wait for 500 ms

    // Decide the best direction based on IR sensors
    if (irLeft == 0 && irRight == 0) { // No walls on either side
      // Choose a random direction (left or right)
      int randomDirection = random(0, 2); // 0 = left, 1 = right
      if (randomDirection == 0) {
        Serial.println("Turning left...");
        turnLeft();
      } else {
        Serial.println("Turning right...");
        turnRight();
      }
    } else if (irLeft == 1 && irRight == 0) { // Wall on the left
      Serial.println("Wall on the left, turning right...");
      turnRight();
    } else if (irLeft == 0 && irRight == 1) { // Wall on the right
      Serial.println("Wall on the right, turning left...");
      turnLeft();
    } else { // Walls on both sides
      Serial.println("Walls on both sides, moving backward...");
      moveBackward();
      delay(500); // Move backward for 500 ms
      turnRight(); // Default to turning right
    }

    delay(500); // Turn for 500 ms
  } else { // If no obstacle is detected in front
      // Decide the best direction based on IR sensors
    if (irLeft == 0 && irRight == 0) { // No walls on either side
      // Choose a random direction (left or right)
      int randomDirection = random(0, 2); // 0 = left, 1 = right
      if (randomDirection == 0) {
        Serial.println("Turning left...");
        turnLeft();
      } else {
        Serial.println("Turning right...");
        turnRight();
      }
    } else if (irLeft == 1 && irRight == 0) { // Wall on the left
      Serial.println("Wall on the left, turning right...");
      turnRight();
    } else if (irLeft == 0 && irRight == 1) { // Wall on the right
      Serial.println("Wall on the right, turning left...");
      turnLeft();
    } else { // Walls on both sides
      Serial.println("Walls on both sides, moving Forward...");
      moveForward();
    }

    delay(500); // Turn for 500 ms
  }
}

// Motor control functions
void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
}

void moveBackward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  delay(500); // Adjust delay for 90-degree turn
  stopMotors();
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  delay(500); // Adjust delay for 90-degree turn
  stopMotors();
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Wall-following adjustments
void adjustLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 100); // Left motor at full speed
  analogWrite(ENB, 80);  // Right motor slightly slower
}

void adjustRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 80);  // Left motor slightly slower
  analogWrite(ENB, 100); // Right motor at full speed
}