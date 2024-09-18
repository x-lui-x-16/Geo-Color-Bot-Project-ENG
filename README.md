# Geo-Color-Bot-Project
Project: “Geo-Color Bot”.
""Si desea esta información en español, simplemente busque en el área de búsqueda de github como: "Geo-Color-Bot-Proyecto-ESP"; o en mi perfil puedes encontrarlo también. 

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


## Electromechanical Components

[Here you will see all the electromechanical components used](electromechanical-components/)

## Energy Management

