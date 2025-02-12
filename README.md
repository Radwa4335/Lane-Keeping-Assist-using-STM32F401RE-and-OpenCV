# Lane Keeping Assist (LKA) System

## Overview
This project implements a **Lane Keeping Assist (LKA) system** using **computer vision-based lane detection** and an **STM32 microcontroller-based motor control system**. The goal is to detect lane deviations and automatically adjust the steering mechanism to maintain the vehicle in the center of the lane.

## Features
- **Lane Detection with Computer Vision**: Utilizes a modified version of [this lane detection repository](https://github.com/canozcivelek/lane-detection-with-steer-and-departure) to detect lane markings and measure deviation.
- **Deviation Transmission via Serial Communication**: The calculated deviation is sent to the STM32 microcontroller through UART.
- **PID-based Steering Correction**: The STM32 microcontroller processes the deviation, applies PID control, and generates motor commands to correct the lane deviation.
- **DC Motor Actuation & Feedback Control**: The motor drives the steering mechanism based on encoder feedback to ensure accurate lane-centering.

## System Components
### **Hardware**
- **Microcontrollers**:
  - STM32F401RE Nucleo Board [(Datasheet)](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
  - STM32F446RE Nucleo Board
  - STM32F401 Black Pill Board
- **Communication Modules**:
  - CAN Transceiver
  - SPI-to-CAN Module (for boards without native CAN support)
- **Steering Mechanism**:
  - DC Motor ([AlveyTech 24V 250W MY1016](https://www.amazon.com/AlveyTech-Motor-Razor-MX350-Pocket/dp/B01MYCRMG4))
  - Optical Encoder (2000 PPR) for angle & velocity feedback
  - Timing pulleys and belts for motion transmission
  - Steering wheel and structural frame
- **Motor Driver**:
  - Cytron MD10-POT ([Specifications](https://store.fut-electronics.com/products/high-power-motor-driver-10a-continuous-15a-peak?srsltid=AfmBOopchVvjWRunJazgWP3_GnZYqnZJgIik5N8agup92ehl0OmJRkHR))
- **Power Supply**:
  - AmpFlow S-400-24 24V 17A ([Specifications](https://www.ampflow.com/))

### **Software**
- **Python-based Lane Detection** (modified from [Original Repo](https://github.com/canozcivelek/lane-detection-with-steer-and-departure))
- **Embedded C-based STM32 Firmware**:
  - UART communication for deviation input
  - PID-based steering control
  - Encoder-based motor feedback

## Installation & Setup
1. **Clone the repository**:
   ```sh
   git clone https://github.com/yourusername/LKA-System.git
   cd LKA-System
   ```
2. **Python Lane Detection Setup**:
   - Install dependencies:
     ```sh
     pip install opencv-python numpy pyserial
     ```
   - Run lane detection:
     ```sh
     python lane_detection.py
     ```
3. **Flash STM32 Firmware**:
   - Compile and flash the firmware to the STM32 board using STM32CubeIDE or PlatformIO.

## Acknowledgments
This project is based on and modifies the lane detection algorithm from [canozcivelek's repository](https://github.com/canozcivelek/lane-detection-with-steer-and-departure). The modifications include **serial communication for deviation transmission** and **PID control for motor-based steering correction**.

## License
This project is open-source, but the lane detection component retains credit to its original author.
