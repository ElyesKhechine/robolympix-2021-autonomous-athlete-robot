# Autonomous Athlete Robot

### RobOlympiX 2021, AerobotiX INSAT Club

**Skills and technologies used:** C++, Arduino, SOLIDWORKS, Control System, Stability, PID, Odometry, I2C, Encoders, Servo Control, Differential Drive, Path Planning

## Introduction

The Autonomous Athlete Robot project, developed for RobOlympiX 2021 in collaboration with AerobotiX INSAT Club, embodies a pioneering effort in creating a 2-wheeled autonomous differential drive robot proficient in shooting hockey pucks and throwing basketballs. Led by a team of 4, the project aimed to revolutionize autonomous navigation algorithms for robotic competitions, demonstrating remarkable advancements in control systems and stability mechanisms.

## Key Features

- Robust cascaded PID controller for precise position, velocity, and orientation tracking of DC motors.
- Implementation of atomic blocks to ensure thread-safe access to shared variables, enhancing encoder reading accuracy.
- Timer-based sampling approach for PID control signals, optimizing responsiveness with a 15-ms sample time.
- Integration of anti-windup schemes to mitigate integral windup effects and ensure stable movement.
- Hardware and software integration of servo motor-based systems for reliable game object manipulation.
- Exploration of self-tuning control algorithms for adaptive PID parameter adjustment based on real-time conditions.

## Project Overview

Executed within a timeline spanning from October 15, 2021, to December 18, 2021, the project underwent meticulous planning, design, and execution phases. Each team member contributed significantly to critical decision-making processes, ensuring the seamless integration of hardware and software components essential for the robot's functionality.

## Technical Details

- **Cascaded PID Controller**: Achieved a 15x improvement in position control accuracy through optimized PID parameters, ensuring precise movement.
- **Atomic Blocks**: Implemented to safeguard critical code sections, guaranteeing uninterrupted encoder readings for enhanced control.
- **Timer-Based Sampling**: Enhanced control loop responsiveness and convergence, facilitating efficient trajectory tracking.
- **Anti-Windup Schemes**: Integrated to mitigate integral windup effects and maintain stability during non-linear control scenarios.
- **Servo Motor Integration**: Ensured seamless interaction with game objects, optimizing performance during competition scenarios.
- **Self-Tuning Control Algorithm**: Explored for adaptive PID parameter adjustment, reducing transient positioning errors by 24%.

## Getting Started

### Installation

1. Assemble hardware components.
2. Upload Arduino code to the microcontroller.
3. Calibrate sensors and fine-tune PID parameters for optimal performance.

### Usage

1. Place the robot on the designated starting point for competition tasks.
2. Power on the robot and observe autonomous navigation capabilities.
3. Monitor performance and make necessary adjustments to achieve desired results.

## Contributing

Contributions aimed at further enhancing project capabilities and performance are welcome.

## License

This project is licensed under the [GPL-3.0 License](LICENSE).

## Contact

For inquiries or feedback, please contact:

- Elyes Khechine: elyeskhechine@gmail.com
- Saif Nbet: saif.nbet@insat.ucar.tn
- Khaled Becher: khaled.becher@insat.ucar.tn
- Helmi Balhoudi: helmi.balhoudi@insat.ucar.tn
