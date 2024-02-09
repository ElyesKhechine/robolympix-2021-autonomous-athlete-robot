# Autonomous Athlete Robot

### RobOlympiX 2021, AerobotiX

**Technologies:** C++, Arduino, SOLIDWORKS, Control System, Stability, PID, Odometry, I2C, Encoders, Servo Control, Differential Drive, Path Planning

## Introduction

The "Autonomous Athlete Robot" project was undertaken as part of the RobOlympiX 2021 competition under the banner of AerobotiX. This endeavor aimed to develop advanced autonomous navigation algorithms for a two-wheeled differential drive robot capable of executing tasks such as shooting hockey pucks and throwing basketballs. 

## Project Scope

 The project was executed between October 15, 2021, and December 18, 2021, involving comprehensive research, development, and testing phases. Led by a team of 4 dedicated individuals, it focused on pushing the boundaries of control systems, stability, and robotics engineering.
 
## Key Features

- **Cascaded PID Controller Optimization**: Implementation of a sophisticated PID controller, leveraging real-time encoder feedback for precise motor control. Achieved a remarkable 15x improvement in position accuracy, reducing errors to a mere 1.9 mm/m.
- **Atomic Blocks for Interrupt Handling**: Implemented atomic blocks to safeguard critical code sections, ensuring uninterrupted operation and accurate encoder readings.
- **Timer-Based Sampling Approach**: Introduced a novel sampling approach for PID control signals, enhancing responsiveness and convergence with an optimized 15-ms sample time.
- **Anti-Windup Schemes**: Integration of anti-windup strategies, including proportional zone jacketing logic, to mitigate integral windup effects and maintain stability under varying conditions.
- **Hardware-Software Integration**: Seamlessly coordinated integration of servo motor-based systems for reliable game object manipulation.
- **Self-Tuning Control Algorithm**: Explored a self-tuning control algorithm to adapt PID parameters in real-time, resulting in a 24% reduction in transient positioning errors.

## Getting Started

### Installation

1. Ensure compatibility and proper setup of hardware components.
2. Install necessary software dependencies, including Arduino IDE and SOLIDWORKS.
3. Configure system settings and upload firmware to the Arduino microcontroller.

### Usage

1. Power on the robot and initialize the control software.
2. Monitor real-time feedback from encoders and sensors using the provided interface.
3. Execute autonomous navigation tasks and observe performance in various scenarios.

## Contributing

Contributions aimed at enhancing project functionalities and addressing emerging challenges are welcome. Please refer to the project's GitHub repository for contribution guidelines.

## License

This project is licensed under the [GPL-3.0 License](LICENSE).

## Contacts

For inquiries or collaboration opportunities, please contact:

- Elyes Khechine: elyeskhechine@gmail.com
- Saif Nbet: saif.nbet@insat.ucar.tn
- Khaled Becher: khaled.becher@insat.ucar.tn
- Helmi Balhoudi: helmi.balhoudi@insat.ucar.tn
