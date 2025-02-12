Lane Keeping Assist (LKA) with STM32 and Vision-Based Lane Detection

Overview
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
This project implements a Lane Keeping Assist (LKA) system that combines computer vision-based lane detection with embedded motor control using an STM32 microcontroller. The system detects lane markings using a modified version of this repository, calculates lane deviation, and transmits it to an STM32F401RE via serial communication. The STM32 executes a PID control loop to correct the vehicle’s deviation by actuating a DC motor, which steers the wheels accordingly.

Features
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
- Lane Detection with OpenCV: Detects lane markings and calculates deviation.
- Serial Communication (UART): Sends deviation data from Python to STM32.
- STM32-Based PID Controller: Adjusts steering to keep the vehicle centered.
- DC Motor Actuation: Uses a Cytron MD10-POT motor driver to control steering.
- Optical Encoder Feedback: Measures steering position and velocity for precise control.


System Architecture
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
1. Lane Detection & Deviation Measurement (Python - OpenCV)
- Uses a modified version of this repository.
- Detects lane boundaries and calculates the lateral deviation from the center.
- Sends the deviation value to the STM32 via serial communication (UART).

2. Microcontroller-Based Steering Control (STM32 - C)
- Receives deviation data from Python via USART2.
- Runs a PID control loop to correct the deviation and maintain lane centering.
- Generates PWM signals to drive the DC motor via the Cytron motor driver.
- Reads feedback from an optical encoder for precise steering adjustments.


Software Implementation
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
1. Lane Detection (Python - OpenCV)
- Preprocesses the video stream.
- Detects lane markings using edge detection and Hough transforms.
- Calculates deviation from the lane center.
- Sends deviation via serial (UART) to STM32.

2. Motor Control (C - STM32)
- Reads deviation from serial input.
- Runs a PID algorithm to compute steering correction.
- Generates PWM signals to control the DC motor.
- Reads encoder feedback to adjust steering.


Setup Instructions
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
1. Software Dependencies
- Python 3.x
- OpenCV (pip install opencv-python)
- PySerial (pip install pyserial)
- STM32CubeIDE for STM32 firmware development

2. Hardware Setup
1. Connect DC motor to Cytron MD10-POT motor driver.
2. Interface STM32F401RE Nucleo with motor driver and encoder.
3. Link Python lane detection script to STM32 via UART.
4. Power the system using 24V 17A power supply.

3. Running the System
1. Running Lane Detection
python lane_detection.py
2. Flashing STM32 Code
- Open STM32CubeIDE.
- Load motor control firmware.
- Flash the code to STM32F401RE.



