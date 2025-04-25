# Non-GPS Navigation for Mobile Platforms with an Event Based Camera
Strathclyde University EEE MEng project

# Overview
This project investigated how a DAVIS346 event-based camera could be used to track the movement of a ground-based rover which could operate in extratterrestrial environemnts.

This repository includes all the code produced throughout the project - for both the event-based camera system, and all control and alternative movement tracking methods implemented in the final system.
![image](https://github.com/user-attachments/assets/52628691-20fc-4982-b893-e0f7037d128a)


# File Structure
## Firmware
Contains all code to program an ESP32. this includes the code for controlling the rover, interfacing with the hardware electronics and determing the rovers displacement using wheel encoders and an IMU.

## Hardware
Contains simulations and schematics of the rover hardware and components

## Software
Contains a CMake project used to program a Raspberry Pi 5.
Includes all camera processing to obtain data from the Davis346 camera, synchronise that data, build frame representations, and integrate it with the open source openVINS system.
![image](https://github.com/user-attachments/assets/7403e51c-b248-40a2-b9f0-bc262eceb490)

Related work:
- https://github.com/uzh-rpg/rpg_dvs_ros
- https://github.com/uzh-rpg/rpg_ultimate_slam_open
- https://github.com/rpng/open_vins

## Testing
Scripts to obtain and plot data from the raspberry Pi 5 logs


