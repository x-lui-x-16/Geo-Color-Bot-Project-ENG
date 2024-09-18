// Pin definitions to control the motor driver
const int pinPWMA = 6;   // Pin for motor A speed (PWM)
const int pinAIN2 = 7;   // Pin for motor A direction (AIN2)
const int pinAIN1 = 8;   // Pin for motor A direction (AIN1)
const int pinBIN1 = 9;   // Pin for motor B direction (BIN1)
const int pinBIN2 = 10;  // Pin for motor B direction (BIN2)
const int pinPWMB = 11;  // Pin for motor B speed (PWM)
const int pinSTBY = 12;  // Pin to enable/disable motors (standby)

// Time and speed variables
const int waitTime = 2000;  // Wait time between phases in milliseconds
const int speed = 0;        // Initial motor speed (can be changed)

// Arrays to group the motor pins
const int pinMotorA[3] = { pinPWMA, pinAIN2, pinAIN1 };  // Motor A pins
const int pinMotorB[3] = { pinPWMB, pinBIN1, pinBIN2 };  // Motor B pins

// Enumeration for movement directions
enum moveDirection {
  forward,     // Move forward
  backward     // Move backward
};

// Enumeration for turn directions
enum turnDirection {
  clockwise,         // Turn clockwise
  counterClockwise   // Turn counterclockwise
};

// Initial setup for pin modes
void setup()
{
  pinMode(pinAIN2, OUTPUT);   // Set AIN2 as output
  pinMode(pinAIN1, OUTPUT);   // Set AIN1 as output
  pinMode(pinPWMA, OUTPUT);   // Set PWMA as output
  pinMode(pinBIN1, OUTPUT);   // Set BIN1 as output
  pinMode(pinBIN2, OUTPUT);   // Set BIN2 as output
  pinMode(pinPWMB, OUTPUT);   // Set PWMB as output
}

// Main loop controlling the vehicle's actions
void loop()
{
  enableMotors();          // Enable motors
  move(forward, 15);       // Move forward at speed 15
  delay(waitTime);         // Wait the defined time

  move(backward, 15);      // Move backward at speed 15
  delay(waitTime);         // Wait

  turn(clockwise, 15);     // Turn clockwise at speed 15
  delay(waitTime);         // Wait

  turn(counterClockwise, 15); // Turn counterclockwise at speed 15
  delay(waitTime);         // Wait

  fullStop();              // Stop the motors
  delay(waitTime);         // Wait
}

// Function to move the vehicle forward or backward
void move(int direction, int speed)
{
  if (direction == forward)  // If direction is forward
  {
    moveMotorForward(pinMotorA, speed);  // Move motor A forward
    moveMotorForward(pinMotorB, speed);  // Move motor B forward
  }
  else  // If direction is backward
  {
    moveMotorBackward(pinMotorA, speed); // Move motor A backward
    moveMotorBackward(pinMotorB, speed); // Move motor B backward
  }
}

// Function to turn the vehicle
void turn(int direction, int speed)
{
  if (direction == clockwise)  // If direction is clockwise
  {
    moveMotorForward(pinMotorA, speed);  // Move motor A forward
    moveMotorBackward(pinMotorB, speed); // Move motor B backward
  }
  else  // If direction is counterclockwise
  {
    moveMotorBackward(pinMotorA, speed); // Move motor A backward
    moveMotorForward(pinMotorB, speed);  // Move motor B forward
  }
}

// Function to completely stop the vehicle
void fullStop()
{
  disableMotors();         // Disable motors
  stopMotor(pinMotorA);    // Stop motor A
  stopMotor(pinMotorB);    // Stop motor B
}

// Function to move a motor forward
void moveMotorForward(const int pinMotor[3], int speed)
{
  digitalWrite(pinMotor[1], HIGH);   // Set motor direction (AIN2/BIN1 to HIGH)
  digitalWrite(pinMotor[2], LOW);    // Set motor direction (AIN1/BIN2 to LOW)

  analogWrite(pinMotor[0], speed);   // Set motor speed (PWM)
}

// Function to move a motor backward
void moveMotorBackward(const int pinMotor[3], int speed)
{
  digitalWrite(pinMotor[1], LOW);    // Set motor direction (AIN2/BIN1 to LOW)
  digitalWrite(pinMotor[2], HIGH);   // Set motor direction (AIN1/BIN2 to HIGH)

  analogWrite(pinMotor[0], speed);   // Set motor speed (PWM)
}

// Function to stop a motor
void stopMotor(const int pinMotor[3])
{
  digitalWrite(pinMotor[1], LOW);    // Set AIN2/BIN1 to LOW (stop)
  digitalWrite(pinMotor[2], LOW);    // Set AIN1/BIN2 to LOW (stop)

  analogWrite(pinMotor[0], 0);       // Set motor speed to 0 (stop)
}

// Function to enable motors
void enableMotors()
{
  digitalWrite(pinSTBY, HIGH);       // Set STBY pin to HIGH to enable motors
}

// Function to disable motors
void disableMotors()
{
  digitalWrite(pinSTBY, LOW);        // Set STBY pin to LOW to disable motors
}