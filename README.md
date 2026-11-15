# Self Leveling Static Tire Balancer

## Senior Design Project: Team 62
 Team name: 3DC

 Project conducted for: Kennesaw State University

This project is for a portable tire balancer. This code relys on a frame structure that is detailed this project that this team designed and built.
The frame uses 3 moment arms to calculate the weight and weight offset to find the center of gravity of the tire mounted.
To ensure the weight offset is accurate, the frame can also self level and take into account the tire number information.

 - Created By: Chance Johnson, Diego Garcia, Daouda Diene, and Destinee Hicks

## How it works

1. The wheel is laid flat on a platform, centred by a jig, resting on three arms 120° apart with a load cell under each.
2. Two linear actuators tilt the platform until an MPU6050 on it reads level. A second MPU6050 on the frame checks the ground is not too steep to correct.
3. The three load cells give the total weight and how far the centre of gravity is from the centre.
4. From the tire size, the machine works out how much weight to add at the rim to bring the centre of gravity back, and a laser on a servo points at where it goes.

An Arduino Nano does the leveling, weighing and laser. An ESP32 touch screen (CYD, JC2432W328C) is the user interface. They talk over serial with short text commands.

## Repository layout

| Folder | What is in it |
|---|---|
| [Nano/TireBalancer_Nano](Nano/TireBalancer_Nano) | Arduino Nano firmware |
| [Screen/ui](Screen/ui) | Touch screen sketch |
| [Screen/libraries](Screen/libraries) | Libraries bundled for the screen (LVGL 8.3.11, TFT_eSPI 2.5.43 with this board's setup, generated UI) |
| [Screen/squareline](Screen/squareline) | SquareLine Studio project for the screens |
| [Tools](Tools) | MPU6050 calibration, I2C scanner, touch test, build script |
| [docs](docs) | [Setup](docs/Setup.md), [Wiring](docs/Wiring.md), [Protocol](docs/Protocol.md), test data, [firmware read back from the boards](docs/firmware-backup) |

See [GOALS.md](GOALS.md) for where the project stands and [CHANGELOG.md](CHANGELOG.md) for history.
Older releases (Nano V0.1-V0.4, screen V0.1-V0.2) are in the git history of `main`.

## Features
 - 2 axis Self Leveling
 - 3 point Tire Weighing
 - 3 axis tire CG (Center of gravity) Calculation
 - Capacitive touchscreen for data entering and viewing
 - Tire centering jig
 - Added weight amount and location output
 - Laser guided pointer to required added weight location
 - Automatic/Manual balancing/leveling are prompted to the user
 - User adjustable constants for individual tuning
 - Open Source Design for user firendly modding
 - lightweight & Foldable frame for easy storage
 - Low Energy (relatively) PD2&3/QC2&3 power supply support
 - Safe due to Static/low motion testing
 - Wide range of tire size support

## Contributions
- Chance Johnson:
  - Application Code Side
    - Leveling calculation
    - Weight location calculation
    - Weight amount calculation
  - HID Code Side
    - Touch Screen Code
    - User interface Graphics
  - Frame Design
  - DFM CAD Model
  - Bill of Materials
  - Frame Assembly
- Diego Garcia:
  - Scheduling
    - Gantt Chart
    - Weekly Schedule
  - Documentation
    - Poster
    - Reports
  - Tire Mounting Mechanism
  - FEA (Simulation Analysis)
- Daouda Diene:
  - Initial Preview CAD Model
- Destinee Hicks:
  - Periodic team participation
- Shared:
  - Expo 1 Presentation
  - Expo 2 Presentation
  - Costs
  - File Review
