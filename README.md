<h1 align="center">GRUZIK3.0: Line Follower Robot</h1>

<div align="center">

![Microcontroller](https://img.shields.io/badge/uC-STM32F103C8T6-white) ![Platform](https://img.shields.io/badge/Platform-STM32cubeIDE-darkcyan) ![Repo Size](https://img.shields.io/github/repo-size/NYDEREK/LineFollower_GRUZIK2.0-light) ![Top Language](https://img.shields.io/github/languages/top/NYDEREK/LineFollower_GRUZIK2.0-light)

<i>Loved the project? Please consider giving a Star ⭐️</i>

</div>

> **GRUZIK2.0 is a lightweight construction (69g) designed for the "Line Follower Light" competition. The robot features Bluetooth communication with a smartphone app and uses a digital sensor instead of analog.**

<div align="center">
  <img src="https://github.com/user-attachments/assets/dc5d3332-ec1f-4ac1-ab5d-26fb2624dbcc" width="700"/>
</div>

## Hardware

> **The simple lightweight construction (69g) allows the robot to achieve high speed on the track.**
> **PCBs were designed in KiCAD 8.0 and manufactured by JLCPCB. The rims were 3D printed with PLA for Kyosho "Mini Z" tires.**
> **The light dualsky 150mAh battery provides enough current (max 30C) without adding too much weight (10g).**
> **The Pololu 2211 motor (10:1) with max current 1.6A per one requires the use of two separate TB6612FNG motor drivers with connected channels (max 2A per driver).**
> **The HC-05 Bluetooth module gives communication range up to 10m.**
> **The fast ARM microcontroller STM32G474RET6 (max 170MHz) provides a lot of software capabilities.**

- uC - STM32G474RET6
- drivers - TB6612FNG
- sensor - 12x KTIR0711S
- battery - 150mAh LiPo dualsky
- wheels - Kyosho Mini Z
- motors - Pololu 3071 HPCB
- Bluetooth - HC-04
- Encoders - AMS AS5047P-ATSM
- IMU - MPU-6050

## 🗺️ Mapping 🗺️
> **GRUZIK3.0 is able to record it's route and save it on uSD card. Results can be represented as XY map and optimized by Python algorithm, robot is able to recreate optimized map in real world achieving max speed for the given route**

<div align="center">
<img src="https://github.com/user-attachments/assets/df6ee6c3-2aab-4b90-810a-3647e3680202" width="700"/>
</div>

## 📱Bluetooth Apps 📱

> **Thease are simple Bluetooth apps created with MIT App Inventor. Spps start and stop the robot. The Modes pad is responsible for quickly adapting the speed and accuracy of the robot to the route using modes, and the application also displays the battery charge status. Second app is mainly for tweaking PID settings it allows you to change any parameter in seconds**

<div align="center">
  <img src="https://github.com/user-attachments/assets/7bbf09f1-de58-4a54-b960-737331f7caec" width="200"/>
  <img src="https://github.com/user-attachments/assets/04cf68f1-61b6-4069-b168-2889a54eaaf8" width="200"/>
</div>
