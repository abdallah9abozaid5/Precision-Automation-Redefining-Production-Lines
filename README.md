# Precision Automation: Smart Sorting Production Line 🤖
This project presents a cost-effective automated production line that integrates a 5-DOF robotic arm, sensor-based decision making, and a conveyor system to perform accurate height-based sorting and pick-and-place operations without human intervention.

The system is designed to overcome limitations of traditional production lines such as high labor costs, human error, and inconsistent sorting accuracy. Using IR sensors, an Arduino-based control system, and a 3D-printed ABS robotic arm, the project achieves reliable, repeatable, and scalable automation suitable for educational and light industrial applications.

🔧 Key Features

5-DOF serial robotic arm with revolute joints

Height-based product classification using IR sensors

Automatic and manual (teaching) operation modes

EEPROM-based motion storage (no reprogramming needed after power-off)

Conveyor belt synchronization and product counting

Lightweight ABS mechanical design optimized through CAD and torque analysis

🧠 System Architecture

The system operates in three main phases:

Sensing: IR sensors detect product presence and height

Processing: Arduino processes sensor data and selects sorting logic

Execution: Robotic arm performs pick-and-place while the conveyor is controlled accordingly

🛠 Technologies Used

Arduino Uno

PCA9685 Servo Driver (I2C)

Servo Motors (MG995, SG90)

SolidWorks (Mechanical Design & Mass Properties)

ABS 3D Printing

EEPROM for motion memory

📌 Applications

Automated sorting systems

Educational robotics projects

Smart manufacturing prototypes

Pick-and-place training platforms

# Developed by Lazy But Smart Bots – Robotics Systems (2025) 🚀
