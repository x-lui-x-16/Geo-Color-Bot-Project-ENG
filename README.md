# Geo-Color-Bot-Project
Project: “Geo-Color Bot”.

[Espanish Version](https://github.com/x-lui-x-16/Geo-Color-Bot-Proyecto-ESP)

## Engineering Materials
All the materials we used for this project. 
- 1 Arduino Mega R3 2560 (https://docs.arduino.cc/hardware/mega-2560/)
- 1 TB6612 H-Bridge (HW-166)
- 1 Bluetooth Module (HC-05)
- 4 Ultrasonic Sensors (HC-SR04) (1 front, 1 back, the other two on the right and left)
- 1 Color Sensor (TCS3200) (In the lower part of the vehicle)
- 1 Case of 2 18650 Batteries
- 2 18650 Batteries (about 7.4 v)
- 1 2wd Chassis - This includes:
    - 1 Acrylic Base
    - 2 Yellow Motors
    - 2 Wheels
    - 1 Crazy Wheel
- ... Protocables
- Multiple 3D parts that can be found in the following directory.
[3D files](3dfiles/)

## Building Instructions
- 2wd Chassis: https://iesjuanlopezmorillas.es/index.php/departamentos/area-cientifico-tecnologica/tecnologia/blog/102-montaje-chasis-coche-2wd
- Electronics (Circuit and Battery): [Circuit diagram can be found in the schematics directory](schemes/)
- Code (For the Arduino): [Code can be found in the src directory](src/)

## Contents
[3D Files](3dfiles/) This folder shows all the 3d files used.

[Electromechanical Components](electromechanical-components/) In this folder are all the electromechanical components used.

[Schemes](schemes/) This folder shows all the project schematics. 

[Code](src/) In this folder you will find the code used.

[Vehicle Photos](v-photos/) In this folder you will find images of the vehicle from different angles.

## Introduction
Below is the development and documentation of the Geo-Color Bot project, an educational robot designed to support learning for people with difficulties such as ADHD or color blindness. The robot uses color detection to associate each color with a specific country, providing relevant information about the country using artificial intelligence.

Project Description:
The Geo-Color Bot is equipped with a TCS3200 color sensor at the base, capable of identifying colors and sending the information to a monitor, where the corresponding country is displayed along with a brief AI-generated description.

For its displacement, the robot has four HC-SR04 ultrasonic sensors, positioned to measure distances in the front, rear, right and left directions, ensuring accurate navigation and obstacle avoidance.

Main Components:
- Microcontroller: Arduino Mega 2560.
- Color sensors: TCS3200
- Ultrasonic sensors: HC-SR04 (x4)
- Wireless communication: Bluetooth module HC-05
- Programming language: C++ (Arduino IDE).

Functionality:
The Arduino Mega 2560 processes the information from the sensors, controls the robot's movements and allows wireless communication with a computer through the Bluetooth module, for remote monitoring and control.

## Objectives.
General Objective:
To develop a robotic educational system that facilitates inclusive and adapted learning for people with learning difficulties, such as ADHD and color blindness, using color sensing technologies and ultrasonic sensors to provide an interactive, effective and entertaining learning experience.

Specific Objectives:
- Design and implement an autonomous robot that uses a TCS3200 color sensor to identify colors and associate them with educational information, such as countries and their characteristics, allowing users to learn in a dynamic and contextualized way.
- Perform accurate programming to control the robot, ensuring that all sensors and motors work in a coordinated and efficient manner. This programming will include algorithms that allow the robot to detect colors, measure distances and make autonomous decisions, making the robot a useful and applicable tool in the real world.
- Integrate HC-SR04 ultrasonic sensors into the robotic system to ensure that the robot can safely navigate, avoid obstacles, and position itself properly to perform color detection in different environments.
- Develop a communication interface using the Bluetooth HC-05 module, which allows transmitting the color data and distances measured by the robot to a computer, facilitating the monitoring and analysis of the information in real time.
- Evaluate the effectiveness of the proposed educational method by testing with users with ADHD and color blindness, collecting data on their learning experience and adjusting the system as necessary to improve its accessibility and effectiveness.
- Promote the use of robotic technologies in the educational setting, demonstrating how interactive and adaptive tools can make learning more accessible and engaging for people with diverse educational needs.

## Beneficiaries and Impact.
Project Beneficiaries:
- People with ADHD (Attention Deficit Hyperactivity Disorder): The Geo-Color Bot is designed to help improve concentration and learning for people with ADHD through an interactive, multi-sensory approach. By integrating visual, auditory and tactile stimuli, it creates a dynamic educational experience that can capture and hold students' attention.
- People with Color Blindness: The project also offers benefits to people with color blindness by facilitating color recognition through association with contextual information, such as country identification. This helps create a more functional understanding of colors in an educational environment.
- Educators and Educational Centers: Educators can use Geo-Color Bot as an inclusive tool to personalize learning to the needs of their students, improving both academic outcomes and overall student well-being.
- Families and Caregivers: Families and caregivers of people with ADHD and color blindness will see improvements in the learning skills and confidence of their loved ones, making this project a useful resource for in-home support as well.

- Robotics Developers and Enthusiasts: Educational technology developers and robotics enthusiasts can benefit from this project as a case study or inspiration for future initiatives, demonstrating how robotics and programming can contribute to inclusive education.

Project Impact:
- Improving Educational Inclusion: the project has a positive impact on inclusive education by adapting teaching methods to the needs of students with ADHD and color blindness, allowing more students to access quality education.
- Promoting Active Learning and Motivation: By incorporating robotics and interactivity into the educational process, the Geo-Color Bot promotes active learning, where students are active participants, which increases motivation and engagement.
- Technical and Cognitive Skills Development: Through interaction with the robot, students not only acquire educational knowledge, but also develop key technical and cognitive skills, such as problem solving and critical thinking.
- Awareness and Social Awareness: The project raises awareness of special educational needs, promoting greater understanding and support towards people with ADHD and color blindness in society.
- Innovation in Education: The Geo-Color Bot represents a breakthrough in educational innovation, demonstrating how technology can be used to address specific and complex challenges in teaching, inspiring new technological solutions in inclusive education.

## Methodology
The development of the Geo-Color Bot focused on several key steps: planning and design, hardware development, software development, integration and testing, evaluation and documentation, and finally, continuous improvement.

Planning and Design:
We defined clear objectives for the project, focusing on accurate color detection and efficient mobility. We selected the most suitable electronic components, such as the TCS3200 color sensor, HC-SR04 ultrasonic sensors to measure distances, and the RP2040 microprocessor, among others.

Hardware development:
The components were integrated into the base of the 2WD robot. The color sensor was placed at the bottom to detect colors on the ground, while the ultrasonic sensors were placed at the front, rear, right and left positions to measure distances. All connections were made so that communication between the sensors and the microprocessor was efficient.
- Project Image:
[Here are some images of the project for better reference:](v-photos/).
- Wiring Diagram:
[Here is how the components are connected:](schemes/)

Software Development:
The code was written in C++ using Arduino IDE. We programmed the color sensor to detect RGB values and the ultrasonic sensors to measure distances in all directions. The software logic allows the robot to navigate autonomously and avoid obstacles, ensuring accurate sensing and mobility.
- Code.
[Here is the code for further understanding:](src/)

Integration and Testing:
This process was divided into three stages:
- System Integration: We connected all the electronic components to the Arduino Nano RP2040.
- Functional Testing: We performed extensive testing and calibration of the ultrasonic sensors and the color sensor to ensure proper operation.
- Adjustments and Optimization: We adjust sensor parameters to optimize navigation and color detection accuracy.

Evaluation and Documentation:
Tests were conducted in a variety of environments to ensure consistency of the robot. We collected data that helped identify areas for improvement. The entire process and test results were documented to facilitate future modifications and improvements.

Continuous Improvement:
We implemented improvements identified during the evaluation phase. These included sensor calibration, adjustments to movement speeds, and refinements to distance and color detection, ensuring that the robot reacts efficiently to different scenarios.

## Expected Results
The following are the main results we expect to achieve with the development of the Geo-Color Bot:

- Color Detection Accuracy:
The robot is expected to detect colors with high accuracy using the TCS3200 sensor, even in different lighting conditions and surfaces. Detection accuracy is crucial for the system to correctly associate each color with its respective educational information, such as country identification. This will ensure that users receive reliable and relevant information, enhancing learning effectiveness.

- Fast and Clear Visualization of Information:
Once a color is detected, the system should display the associated educational information quickly and clearly on a screen. The response time shall be short so that educational data is presented almost instantaneously after color detection. This is critical to maintain a continuous educational flow and a smooth user experience.

- Efficient Mobility and Collision Avoidance:
Thanks to the HC-SR04 ultrasonic sensors, the robot will move efficiently, detecting and avoiding obstacles in all directions (forward, backward, right and left). This will allow the robot to operate autonomously and safely, without risk of collisions, guaranteeing the integrity of the robot and its environment, and prolonging its useful life.

- Educational and Engaging Interaction:
The robot will provide an interactive and engaging educational experience, designed especially to engage the interest of people with learning difficulties, such as ADHD and color blindness. The combination of multi-sensory and visual learning will motivate users, making the teaching process dynamic, fun and effective.

- Reliability and Stability in Prolonged Use:
The system will exhibit high reliability and stability during periods of prolonged use. Even after several hours of continuous operation, the robot will maintain its performance without technical errors or failures, ensuring its applicability in real educational environments.

- Continuous Improvement Capability:
The robot design will allow for future improvements based on collected data and user feedback. This includes the ability to adjust programming to improve color detection, optimize mobility and refine the educational experience. This adaptability will ensure the constant evolution of the robot in the face of new needs.

- Adaptability to New Conditions:
The robot will be able to adapt to different environmental conditions, such as changes in lighting or space layout. This versatility will allow it to be useful in diverse environments, from classrooms with variable configurations to educational events.

- Incorporation of Future Functionalities:
The system will be designed with flexibility to allow for the integration of new functionality in the future, such as the incorporation of other sensors, improvements to the user interface, or expansion into other educational disciplines. This expandability will ensure the longevity of the project and increase its impact.

## Budget
The following is the estimated budget for the development of the Geo-Color Bot project:

- Arduino Mega 2560 - $57.31
- TCS3200 Color Sensor - $9.49
- Ultrasonic Sensor HC-SR04 (x4) - $22.00 ($5.50 ea.)
- TB6612 H-Bridge - $3.50
- Protoboard - $3.00
- Arduino 2WD Kit - $10.00
    - Includes:
    - Acrylic Base
    - Yellow Motor
    - Wheel
    - Crazy Wheel
- 3D Printing Base for Ultrasonic Sensors (x2) - Donation
- Case of 2 18650 Batteries - $3.00
- 3.7V 18650 Batteries (x2) - $7.00 ($3.50 each)
- Pack of 10 Dupont Male-Male Cables (x3) - $4.50 ($1.50 ea.)
- Pack of 10 Dupont Male-Female Cables (x2) - $3.00 ($1.50 ea.)
- Soldering Station with Soldering Iron - $25.95
- Tin - $12.30
- Heat Shrink Box - $6.50
- Wire Stripper - $4.29
- Pliers - $2.50
- Cutting Pliers - $3.00
- Lighter - $2.00
- Transparent Tape - $0.50
- Double Contact Tape - $7.00
- Super 33 Vinyl Electrical Tape - $3.00
- Strapping Pack (x2) - $7.00 ($3.50 each)

Total - $196.94

## Conclusion
This project has managed to develop an innovative educational tool, focused on improving learning for people with ADHD and color blindness. Thanks to the integration of the TCS3200 color sensor and the HC-SR04 ultrasonic sensors, the robot is capable of detecting colors with great precision and navigating safely, providing an interactive and accessible educational experience.

The design ensures reliability and stability during long-term use, allowing its application in educational environments without interruption. Furthermore, its ability to adapt and continuously improve ensures that this project remains relevant and effective in the long term.

In summary, the Geo-Color Bot represents a significant step forward toward inclusive education, providing more dynamic and personalized learning for users facing challenges in color perception and attention.

## Code Explanation
### Main Code
[Main Code](src/main/)

This code is designed to drive a robot using several ultrasonic sensors and a TCS3200 color sensor, controlling the motion with the TB6612 motor controller. Below, I explain each part of the code:

#### Pin Definition

// Color Sensor (TCS3200) Pins Definition

#define OUT 2

#define S0 3

#define S1 4

#define S2 5

#define S3 6

#define LED 7


The pins for connecting the TCS3200 color sensor are defined. These pins control the configuration of the sensor to read the different color values and activate its LED:
- OUT is the pin where the sensor pulse is received, which varies depending on the detected color.
- S0, S1, S2, S3 control the output frequency of the sensor and select which color component (red, green, blue) to measure.
- LED turns on the sensor's LED light.


// US Sensor Pins Definition (HC-SR04)

#define ECHO1 22 // Front sensor

#define TRIG1 23

#define ECHO2 30 // Left Sensor

#define TRIG2 31

#define ECHO3 36 // Right sensor

#define TRIG3 37

#define ECHO4 46 // Rear Sensor

#define TRIG4 47


The pins for the ultrasonic sensors (HC-SR04) are defined here. Each sensor has two main pins:
- TRIG: Pin to send the trigger signal.
- ECHO: Pin to receive the echo, which allows to calculate the distance.


// Controller Pins Definition (TB6612)

#define PWMA 53

#define AIN1 54

#define AIN2 55

#define BIN1 57

#define BIN2 58

#define PWMB 59

#define STBY 70


The pins for controlling the TB6612 motor driver are defined. These pins allow to control the direction and speed of the robot motors:
- PWMA and PWMB control the speed of the motors.
- AIN1, AIN2, BIN1, BIN2 control the direction of the motors.
- STBY brings the controller out of standby mode, allowing the motors to run.


#### Variables for timing control

const unsigned long printInterval = 5000; // Interval of 5 seconds

unsigned long previousMillis = 0;


These variables are used to control the time interval between sensor readings. printInterval defines an interval of 5 seconds, while previousMillis stores the time when the last reading was taken.

#### setup() function

void setup() {

  Serial.begin(9600); // Start Serial communication
  

Here the serial communication is started at a rate of 9600 baud, which will allow data to be sent to the serial monitor.

##### Color sensor configuration

  pinMode(S0, OUTPUT);
  
  pinMode(S1, OUTPUT);
  
  pinMode(S2, OUTPUT);
  
  pinMode(S3, OUTPUT);
  
  pinMode(OUT, INPUT);
  
  pinMode(LED, OUTPUT);
  
  
  digitalWrite(S0, HIGH);

  
  digitalWrite(S1, LOW);
  
  digitalWrite(LED, HIGH);
  

The color sensor pins are configured as inputs or outputs as needed. S0 and S1 control the output frequency, which in this case is set to 100%. The sensor LED lights up to illuminate the area being detected.

##### Ultrasonic sensor configuration

  pinMode(TRIG1, OUTPUT);
  
  pinMode(ECHO1, INPUT);
  
  pinMode(TRIG2, OUTPUT);
  
  pinMode(ECHO2, INPUT);
  
  pinMode(TRIG3, OUTPUT);
  
  pinMode(ECHO3, INPUT);
  
  pinMode(TRIG4, OUTPUT);
  
  pinMode(ECHO4, INPUT);
  

Each ultrasonic sensor has a TRIG pin to send the signal and an ECHO pin to receive the reflected signal. The pins for these sensors are configured here.

##### TB6612 controller configuration

  pinMode(AIN2, OUTPUT);
  
  pinMode(AIN1, OUTPUT);
  
  pinMode(PWMA, OUTPUT);
  
  pinMode(BIN1, OUTPUT);
  
  pinMode(BIN2, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  
  pinMode(STBY, OUTPUT);
  
  digitalWrite(STBY, HIGH);
  

The motor controller pins are configured as outputs and the controller is brought out of standby mode by activating the STBY pin.

#### Loop() function

void loop() {

  unsigned long currentMillis = millis();
  
  
  if (currentMillis - previousMillis >= printInterval) {
  
    previousMillis = currentMillis;
    

In the main loop (loop()), it is checked if 5 seconds (printInterval) have passed since the last time the data was printed. If so, the timer is updated and new readings are taken.

#### Get and display sensor data

  int distanceFront = measureDistance(TRIG1, ECHO1);
  
  int distanceLeft = measureDistance(TRIG2, ECHO2);
  
  int distanceRight = measureDistance(TRIG3, ECHO3);
  
  int distanceBack = measureDistance(TRIG4, ECHO4);
  
  int redValue, greenValue, blueValue;
  
  readColor(&redValue, &greenValue, &blueValue);
  

The functions measureDistance and readColor are called to obtain the distances of the ultrasonic sensors and the color values of the TCS3200 sensor, respectively.


  Serial.print(“Distance front: ‘); Serial.print(distanceFront); Serial.println(’ cm”);
  
  Serial.print(“Distance left: ‘); Serial.print(distanceLeft); Serial.println(’ cm”);
  
  Serial.print(“Distance right: ‘); Serial.print(distanceRight); Serial.println(’ cm”);
  
  Serial.print(“Distance back: ‘); Serial.print(distanceBack); Serial.println(’ cm”);
  
    
  Serial.print("Color (RGB): ”);
  
  Serial.print("R: ”); Serial.print(redValue);
  
  Serial.print(” G: ”); Serial.print(greenValue);
  
  Serial.print(” B: ”); Serial.println(blueValue);
  
    
  Serial.println(“----------------------------”);
  
  }
  
}


The results of the distance and color measurements are printed on the serial monitor, showing the distances and the RGB values of the color sensor.

#### measureDistance() function

int measureDistance(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  
  delayMicroseconds(10);
  
  digitalWrite(trigPin, LOW);
  
  
  long duration = pulseIn(echoPin, HIGH);
  
  int distance = duration * 0.034 / 2;
  
  return distance;
  
}


This function measures the distance using an ultrasonic sensor. A short pulse is sent from the TRIG pin, and then the time it takes to receive the echo on the ECHO pin is measured. The distance is calculated based on that time.

#### readColor() function

void readColor(int* redValue, int* greenValue, int* blueValue) {


  // Read red value
  
  digitalWrite(S2, LOW);
  
  digitalWrite(S3, LOW);
  
  delay(100);
  
  *redValue = pulseIn(OUT, LOW);
  

  // Read green value
  
  digitalWrite(S2, HIGH);
  
  digitalWrite(S3, HIGH);
  
  delay(100);
  
  *greenValue = pulseIn(OUT, LOW);
  

  // Read blue value
  
  digitalWrite(S2, LOW);
  
  digitalWrite(S3, HIGH);
  
  delay(100);
  
  *blueValue = pulseIn(OUT, LOW);
  
}


This function reads the color values (red, green, blue) from the TCS3200 sensor. Depending on the configuration of pins S2 and S3, the sensor detects one of the three colors and sends a pulse whose duration is measured with pulseIn() to obtain the RGB values.


### Mobility
[Mobility code](src/parts-of-code/Mobility.ino)

I explain each part of the code:

#### Pin definition.

#define pinPWMA 54

#define pinAIN2 55

#define pinAIN1 56

#define pinBIN1 58

#define pinBIN2 59

#define pinPWMB 60

#define pinSTBY 61

- pinPWMA**: Speed pin for motor A. The TB6612 controller controls the speed with a PWM (pulse width modulation) signal.
- pinAIN1, pinAIN2**: Pins that control the direction of motor A. Depending on how they are set, the motor will go forward or backward.
- pinPWMB**: Speed pin for motor B.
- pinBIN1, pinBIN2**: Pins that control the direction of motor B, similar to those of A.
- pinSTBY**: Pin that activates or deactivates the motors (standby).

#### Time and speed variables

const int waitTime = 2000;

const int speed = 0;

- waitTime**: Waiting time in milliseconds between each action.
- **speed**: Initial speed value (here it is 0, but it can be changed in the loop).

#### Motor pin arrays

const int pinMotorA[3] = { pinPWMA, pinAIN2, pinAIN1 };

const int pinMotorB[3] = { pinPWMB, pinBIN1, pinBIN2 };

- These arrays group the pins of each motor to make it easier to pass the pins as arguments to the functions that control them.

#### Enumerations for movement

enum moveDirection {

  forward,  
  
  backward   
  
};

enum turnDirection {

  clockwise,     
  
  counterClockwise   
  
};


- moveDirection**: Defines the movement directions: forward and backward.
- turnDirection**: Defines the turns: clockwise and counterClockwise.

#### Initial configuration

void setup() {

  pinMode(pinAIN2, OUTPUT);
  
  pinMode(pinAIN1, OUTPUT);
  
  pinMode(pinPWMA, OUTPUT);
  
  pinMode(pinBIN1, OUTPUT);
  
  pinMode(pinBIN2, OUTPUT);
  
  pinMode(pinPWMB, OUTPUT);
  
}


- All pins associated with the motors are configured as outputs.

#### Main loop

void loop() {

  enableMotors();  
  
  move(forward, 250);  
  
  delay(waitTime);    
  

  move(backward, 250);     
  
  delay(waitTime);       
  

  // Read red value
  
  digitalWrite(S2, LOW);
  
  digitalWrite(S3, LOW);
  
  delay(100);
  
  *redValue = pulseIn(OUT, LOW);
  

  // Read green value
  
  digitalWrite(S2, HIGH);
  
  digitalWrite(S3, HIGH);
  
  delay(100);
  
  *greenValue = pulseIn(OUT, LOW);
  

  // Read blue value
  
  digitalWrite(S2, LOW);
  
  digitalWrite(S3, HIGH);
  
  delay(100);
  
  *blueValue = pulseIn(OUT, LOW);
  
}


This function reads the color values (red, green, blue) from the TCS3200 sensor. Depending on the configuration of pins S2 and S3, the sensor detects one of the three colors and sends a pulse whose duration is measured with pulseIn() to obtain the RGB values.


### Mobility
[Mobility code](src/parts-of-code/Mobility.ino)

I explain each part of the code:

#### Pin definition.

#define pinPWMA 54

#define pinAIN2 55

#define pinAIN1 56

#define pinBIN1 58

#define pinBIN2 59

#define pinPWMB 60

#define pinSTBY 61

- pinPWMA**: Speed pin for motor A. The TB6612 controller controls the speed with a PWM (pulse width modulation) signal.
- pinAIN1, pinAIN2**: Pins that control the direction of motor A. Depending on how they are set, the motor will go forward or backward.
- pinPWMB**: Speed pin for motor B.
- pinBIN1, pinBIN2**: Pins that control the direction of motor B, similar to those of A.
- pinSTBY**: Pin that activates or deactivates the motors (standby).

#### Time and speed variables

const int waitTime = 2000;

const int speed = 0;

- waitTime**: Waiting time in milliseconds between each action.
- **speed**: Initial speed value (here it is 0, but it can be changed in the loop).

#### Motor pin arrays

const int pinMotorA[3] = { pinPWMA, pinAIN2, pinAIN1 };

const int pinMotorB[3] = { pinPWMB, pinBIN1, pinBIN2 };

- These arrays group the pins of each motor to make it easier to pass the pins as arguments to the functions that control them.

#### Enumerations for movement

enum moveDirection {

  forward,  
  
  backward   
  
};

enum turnDirection {

  clockwise,     
  
  counterClockwise   
  
};


- moveDirection**: Defines the movement directions: forward and backward.
- turnDirection**: Defines the turns: clockwise and counterClockwise.

#### Initial configuration

void setup() {

  pinMode(pinAIN2, OUTPUT);
  
  pinMode(pinAIN1, OUTPUT);
  
  pinMode(pinPWMA, OUTPUT);
  
  pinMode(pinBIN1, OUTPUT);
  
  pinMode(pinBIN2, OUTPUT);
  
  pinMode(pinPWMB, OUTPUT);
  
}


- All pins associated with the motors are configured as outputs.

#### Main loop

void loop() {

  enableMotors();  
  
  move(forward, 250);  
  
  delay(waitTime);    
  

  move(backward, 250);     
  
  delay(waitTime);       
  
  turn(clockwise, 250);    
  
  delay(waitTime); 
  

  turn(counterClockwise, 250); 
  
  delay(waitTime);        
  

  fullStop();            
  
  delay(waitTime);         
  
}


- **enableMotors()**: Enables the motors.
- **move(forward, 250)**: Moves the robot forward with a speed of 250 (on a PWM scale).
- move(backward, 250)**: Moves the robot backward.
- turn(clockwise, 250)**: Rotates the robot clockwise.
- turn(counterClockwise, 250)**: Rotates counterclockwise.
- fulllStop()**: Stops all motors.

#### Function to move the vehicle forward or backward

void move(int direction, int speed) {

  if (direction == forward) {
  
    moveMotorForward(pinMotorA, speed);
    
    moveMotorForward(pinMotorB, speed);
    
  } else {
  
    moveMotorBackward(pinMotorA, speed);
    
    moveMotorBackward(pinMotorB, speed);
    
  }
  
}


- Depending on the direction (forward or backward), the corresponding functions are called to move both motors.

#### Function to turn

void turn(int direction, int speed) {

  if (direction == clockwise) {
  
    moveMotorForward(pinMotorA, speed);
    
    moveMotorBackward(pinMotorB, speed);
    
  } else {
  
    moveMotorBackward(pinMotorA, speed);
    
    moveMotorForward(pinMotorB, speed);
    
  }
  
}


- To turn, one motor moves forward and the other moves backward, depending on the direction of the turn.

#### Function to stop the vehicle

void fullStop() {

  disableMotors();
  
  stopMotor(pinMotorA);
  
  stopMotor(pinMotorB);
  
}


- Calls **disableMotors()** to disable the motors and **stopMotor()** to stop each motor.

#### Functions to move the motors

void moveMotorForward(const int pinMotor[3], int speed) {

  digitalWrite(pinMotor[1], HIGH);  
  
  digitalWrite(pinMotor[2], LOW);   
  
  analogWrite(pinMotor[0], speed);  
  
}


void moveMotorBackward(const int pinMotor[3], int speed) {

  digitalWrite(pinMotor[1], LOW);   
  
  digitalWrite(pinMotor[2], HIGH);  
  
  analogWrite(pinMotor[0], speed);  
  
}


- These functions control the motor individually, setting the direction pins and speed as needed.

#### Function to stop a motor

void stopMotor(const int pinMotor[3]) {

  digitalWrite(pinMotor[1], LOW);   
  
  digitalWrite(pinMotor[2], LOW);   
  
  analogWrite(pinMotor[0], 0);    
  
}


- Completely stops a motor by setting the speed to 0 and turning off the direction pins.

#### Enable and disable motors

void enableMotors() {

  digitalWrite(pinSTBY, HIGH);    
  
}

void disableMotors() {

  digitalWrite(pinSTBY, LOW);      
  
}


- **enableMotors()**: Enables the motors when disabling standby mode.
- **disableMotors()**: Disables the motors by activating the standby mode.

This code provides complete control over the motors, with functions to move the robot forward and backward, turn and stop.

#### Code Description

This code controls two motors using a dual motor controller (such as the TB6612), assigning Arduino pins to manage the speed and direction of each motor. Here is how it is structured:

1. Pin Definition.
Pins are defined to control the speed (PWM) and direction (AIN1, AIN2, BIN1, BIN2) of the A and B motors. In addition, a pin is assigned to control the state of the motors (standby).
2. Variables and Arrays
waitTime sets the waiting time between each motion phase.
speed defines the initial speed of the motors, which can be modified in the code.
The pinMotorA and pinMotorB arrays group the pins of each motor, facilitating their control in the functions.
3. Enumerations for Motion and Rotation
Enumerations (enum) are used to define the possible directions of movement (forward, backward) and rotation (clockwise, counterclockwise), making the code more readable and easier to understand.
4. Setup() function
Here you configure the output pins to control the motors. The direction and speed pins are set up to send signals to the motors.
5. Loop() function
This function is the main loop of the program, where the following actions are executed:
Enable the motors using the enableMotors() function.
Move forward at speed 15.
Move backwards at speed 15.
Rotate clockwise at speed 15.
Rotate counterclockwise at speed 15.
Stop the motors using the fullStop() function.
Each action is separated by a delay (delay(waitTime)) to observe the behavior of the motors before moving to the next action.
6. Motion Control Functions
move(direction, speed): Moves the motors forward or backward depending on the indicated direction.
turn(direction, speed): Turns the vehicle in the indicated direction (clockwise or counterclockwise), moving one motor forward and the other backward.
7. Auxiliary Functions
moveMotorForward(pinMotor, speed): Controls a motor to turn forward, setting the direction and speed by PWM.
moveMotorBackward(pinMotor, speed): Controls a motor to turn backward.
stopMotor(pinMotor): Stops the motor by lowering the speed to 0 and setting both direction pins to LOW.
enableMotors() and disableMotors(): Enable or disable the motors by controlling the standby pin (pinSTBY).

#### Code Description

This code controls two motors using a dual motor driver (such as the TB6612), assigning Arduino pins to manage the speed and direction of each motor. Below is a detailed explanation of how it is structured:

1. Pin Definition Pins are defined to control the speed (PWM) and direction (AIN1, AIN2, BIN1, BIN2) of motors A and B. Additionally, a pin is assigned to control the motor state (standby).
2. Variables and Arrays waitTime sets the waiting time between each movement phase. speed defines the initial speed of the motors, which can be modified in the code. The pinMotorA and pinMotorB arrays group the pins of each motor, making their control easier in the functions.
3. Enumerations for Movement and Turning Enumerations (enum) are used to define possible movement directions (forward, backward) and turning directions (clockwise, counterclockwise), making the code more readable and easier to understand.
4. setup() Function Here, the output pins are configured to control the motors. The direction and speed pins are prepared to send signals to the motors.
5. loop() Function This function is the main loop of the program, where the following actions are executed: Activate the motors using the enableMotors() function. Move forward at speed 15. Move backward at speed 15. Turn clockwise at speed 15. Turn counterclockwise at speed 15. Stop the motors using the fullStop() function. Each action is separated by a delay (delay(waitTime)) to observe the behavior of the motors before moving to the next action.
6. Movement Control Functions move(direction, speed): Moves the motors forward or backward depending on the indicated direction. turn(direction, speed): Turns the vehicle in the indicated direction (clockwise or counterclockwise), moving one motor forward and the other backward.
7. Auxiliary Functions moveMotorForward(pinMotor, speed): Controls a motor to rotate forward, setting the direction and speed using PWM. moveMotorBackward(pinMotor, speed): Controls a motor to rotate backward. stopMotor(pinMotor): Stops the motor by lowering the speed to 0 and setting both direction pins to LOW. enableMotors() and disableMotors(): Activate or deactivate the motors by controlling the standby pin (pinSTBY).

#### Summary

This code offers basic yet flexible control for a two-motor system, allowing the vehicle to move forward, backward, and perform turns. The functions are organized so that it is easy to modify the speed, direction, and overall behavior of the system, making it ideal for basic mobile robots or Arduino motor control projects.

## Electromechanical Components

[Here you will see all the electromechanical components used](electromechanical-components/)

## Energy Management

