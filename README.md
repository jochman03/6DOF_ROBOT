# 6-DOF Robot Project (Currently 4-DOF)

## Overview

Firmware and simulation project for a robotic arm currently implemented with 4 degrees of freedom.
The robot uses live inverse kinematics computed directly on an STM32 microcontroller and can be controlled using a joystick or UART commands.

The latest version replaces floating-point calculations with fixed-point arithmetic and LUT-based trigonometric functions to improve performance and motion smoothness on embedded hardware.

## Tech Stack

- STM32
- Embedded C
- MATLAB
- Joystick controller
- UART commands
- OLED display

## Gallery

### Trajectory test

![Trajectory test](simulations/ik_test_1.gif)

### Videos

https://www.youtube.com/playlist?list=PLLbGYqHAyf1fbl_TrEpRQ8hpMJwUmI0mh
