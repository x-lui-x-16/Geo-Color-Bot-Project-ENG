// Color Sensor Pins Definition (TCS3200)
#define OUT 2
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define LED 7

// Ultrasonic Sensor Pins Definition (HC-SR04)
#define ECHO1 22 // Front sensor
#define TRIG1 23
#define ECHO2 30 // Left sensor
#define TRIG2 31
#define ECHO3 36 // Right sensor
#define TRIG3 37
#define ECHO4 46 // Rear sensor
#define TRIG4 47

// Motor Controller Pins Definition (TB6612)
#define PWMA 54
#define AIN1 54 
#define AIN2 55
#define BIN1 57
#define BIN2 58
#define PWMB 59
#define STBY 70

// Global variables for distances and color
int distanceFront, distanceRight, distanceLeft, distanceBack;
int redValue, greenValue, blueValue;
unsigned long lastPrintTime = 0;  // Variable for time control
const unsigned long printInterval = 5000;  // 5-second interval

// Time and speed variables
const int waitTime = 2000;  // Wait time between phases in milliseconds
const int speed = 0;    // Initial motor speed (can be modified)

// Arrays to group the pins of each motor
const int pinMotorA[3] = { PWMA, AIN2, AIN1 };  // Motor A pins
const int pinMotorB[3] = { PWMB, BIN1, BIN2 };  // Motor B pins

// Enumeration for movement directions
enum moveDirection {
  forward,     // Move forward
  backward     // Move backward
};

// Enumeration for turning directions
enum turnDirection {
  clockwise,         // Turn clockwise
  counterClockwise   // Turn counterclockwise
};

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set up color sensor pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  pinMode(LED, OUTPUT);
  
  // Set sensor frequency to 100%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  // Turn on the sensor LED
  digitalWrite(LED, HIGH);

  // Setup for HC-SR04 pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  pinMode(TRIG4, OUTPUT);
  pinMode(ECHO4, INPUT);

  // Setup for TB6612 pins
  pinMode(AIN2, OUTPUT);   // Set AIN2 as output
  pinMode(AIN1, OUTPUT);   // Set AIN1 as output
  pinMode(PWMA, OUTPUT);   // Set PWMA as output
  pinMode(BIN1, OUTPUT);   // Set BIN1 as output
  pinMode(BIN2, OUTPUT);   // Set BIN2 as output
  pinMode(PWMB, OUTPUT);   // Set PWMB as output
  pinMode(STBY, OUTPUT);   // Configure the standby pin
  digitalWrite(STBY, HIGH); // Activate the TB6612 controller (disable standby)
}

// Function to read color values
void readColor() {
  // Read the red value
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(100); // Wait for the sensor to stabilize
  redValue = pulseIn(OUT, LOW);
  
  // Read the green value
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  delay(100); // Wait for the sensor to stabilize
  greenValue = pulseIn(OUT, LOW);
  
  // Read the blue value
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  delay(100); // Wait for the sensor to stabilize
  blueValue = pulseIn(OUT, LOW);
}

// Function to measure distance with the HC-SR04
int measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;  // Convert to distance
  return distance;
}

String identifyColor(int red, int green, int blue) {
  // Detect colors based on specific ranges of RGB values
  if (red >= 38 && red <= 42 && green >= 25 && green <= 29 && blue >= 17 && blue <= 21) {
    return "Cyan";
  } else if (red >= 34 && red <= 38 && green >= 43 && green <= 47 && blue >= 25 && blue <= 29) {
    return "Magenta";
  } else if (red >= 38 && red <= 42 && green >= 28 && green <= 32 && blue >= 32 && blue <= 36) {
    return "Green";
  } else if (red >= 16 && red <= 20 && green >= 26 && green <= 30 && blue >= 28 && blue <= 33) {
    return "Orange";
  } else if (red >= 44 && red <= 47 && green >= 45 && green <= 51 && blue >= 28 && blue <= 34) {
    return "Violet";
  } else if (red >= 62 && red <= 68 && green >= 27 && green <= 32 && blue >= 25 && blue <= 30) {
    return "Blue";
  } else if (red >= 18 && red <= 21 && green >= 28 && green <= 32 && blue >= 27 && blue <= 31) {
    return "Yellow";
  } else if (red >= 22 && red <= 27 && green >= 49 && green <= 57 && blue >= 36 && blue <= 42) {
    return "Red";
  }
  return "Undefined";
}

void describeCountry(String color) {
  if (color == "Red") {
    Serial.println("Country: China - Description: China is the most populous country in the world and has a rich cultural and economic history.");
  } else if (color == "Green") {
    Serial.println("Country: Brazil - Description: Brazil is the largest country in South America, known for the Amazon and Carnival.");
  } else if (color == "Blue") {
    Serial.println("Country: France - Description: France is known for its art, history, and as a global fashion center.");
  } else if (color == "Cyan") {
    Serial.println("Country: Bahamas - Description: The Bahamas is an island nation known for its beautiful white-sand beaches.");
  } else if (color == "Magenta") {
    Serial.println("Country: India - Description: India is famous for its rich culture, traditions, and diversity of people.");
  } else if (color == "Yellow") {
    Serial.println("Country: Spain - Description: Spain is known for its cuisine, historical architecture, and flamenco.");
  } else if (color == "Orange") {
    Serial.println("Country: Netherlands - Description: The Netherlands is famous for its tulips, windmills, and canals.");
  } else if (color == "Violet") {
    Serial.println("Country: Japan - Description: Japan is known for its advanced technology, traditional culture, and anime.");
  } else {
    Serial.println("Color not identified.");
  }
}

void loop() {
  // Take two initial sensor readings
  for (int i = 0; i < 1000; i++) {
    makeReading();
    delay(printInterval);  // Wait 10 seconds between readings
  }

  // Move forward until detecting 6 cm or less on the front sensor
  while (measureDistance(TRIG1, ECHO1) > 6 || measureDistance(TRIG1, ECHO1) < 4) {
    move(forward, 180);  // Move at speed 180
  }
  fullStop();

  // Take two readings after reaching 6 cm
  for (int i = 0; i < 1000; i++) {
    makeReading();
    delay(printInterval);  // Wait 10 seconds between readings
  }

  // Decide which way to turn based on the lateral sensor readings
  int distanceLeft = measureDistance(TRIG2, ECHO2);
  int distanceRight = measureDistance(TRIG3, ECHO3);
  
  if (distanceLeft > distanceRight) {
    turn(counterClockwise, 180);  // Turn left
  } else {
    turn(clockwise, 180);  // Turn right
  }
  
  // Move forward until the distances in front and back are equal
  while (measureDistance(TRIG1, ECHO1) != measureDistance(TRIG4, ECHO4)) {
    move(forward, 180);
  }
  fullStop();
  
  // Repeat the sequence
}

// Function to take a reading of all sensors and the color
void makeReading() {
  // Read distances from all ultrasonic sensors
  distanceFront = measureDistance(TRIG1, ECHO1);
  distanceLeft = measureDistance(TRIG2, ECHO2);
  distanceRight = measureDistance(TRIG3, ECHO3);
  distanceBack = measureDistance(TRIG4, ECHO4);
  
  // Read color
  readColor();
  
  // Identify color and describe associated country
  String detectedColor = identifyColor(redValue, greenValue, blueValue);
  describeCountry(detectedColor);
  
  // Print sensor and color data to Serial
  Serial.print("Front: "); Serial.print(distanceFront); Serial.print(" cm, ");
  Serial.print("Back: "); Serial.print(distanceBack); Serial.print(" cm, ");
  Serial.print("Left: "); Serial.print(distanceLeft); Serial.print(" cm, ");
  Serial.print("Right: "); Serial.print(distanceRight); Serial.print(" cm, ");
  Serial.print("Red: "); Serial.print(redValue); Serial.print(", ");
  Serial.print("Green: "); Serial.print(greenValue); Serial.print(", ");
  Serial.print("Blue: "); Serial.print(blueValue); Serial.print(" - ");
  Serial.print("Color: "); Serial.println(detectedColor);
  Serial.print("Color (RGB): ");    
  Serial.print("R: "); Serial.print(redValue);
  Serial.print(" G: "); Serial.print(greenValue);
  Serial.print(" B: "); Serial.println(blueValue);
}

// Function to move the vehicle forward or backward
void move(int direction, int speed)
{
  if (direction == forward)  // If the direction is forward
  {
    moveMotorForward(pinMotorA, speed);  // Move motor A forward
    moveMotorForward(pinMotorB, speed);  // Move motor B forward
  }
  else  // If the direction is backward
  {
    moveMotorBackward(pinMotorA, speed); // Move motor A backward
    moveMotorBackward(pinMotorB, speed); // Move motor B backward
  }
}

// Function to turn the vehicle
void turn(int direction, int speed)
{
  if (direction == clockwise)  // If the direction is clockwise
  {
    moveMotorForward(pinMotorA, speed);  // Move motor A forward
    moveMotorBackward(pinMotorB, speed); // Move motor B backward
  }
  else  // If the direction is counterclockwise
  {
    moveMotorBackward(pinMotorA, speed); // Move motor A backward
    moveMotorForward(pinMotorB, speed);  // Move motor B forward
  }
}

// Function to fully stop the vehicle
void fullStop()
{
  disableMotors();         // Disable the motors
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

// Function to enable the motors
void enableMotors()
{
  digitalWrite(STBY, HIGH);       // Set STBY pin to HIGH to enable the motors
}

// Function to disable the motors
void disableMotors()
{
  digitalWrite(STBY, LOW);        // Set STBY pin to LOW to disable the motors
}
