# McGill Rocket Team High-Altitude Balloon
## Introduction
Here is the GitHub repo for McGill Rocket Team High-Alititude Balloon (MRT-HAB). We design, test, and launch a high-altitude balloon to provide a test platform for other subteams and projects in MRT. We have three divisions: Structure, Electronic, and Software.

We mainly put our software development work here. We have a flight computer program, a radio system, a user interface (UI), and some simulations. The project description, current progress, and to-do list are listed below.

## Project description
### Design Goal/Requirements
As mentioned before, our responsibility is to provide a test platform. Hence, our design goal for the structure must be compatible with different payloads. We design a modular inner structure for containing our clients' payload and our own electronic devices.

Also, we need to ensure the balloon can be terminated after reaching the desired altitude, i.e., separate the balloon from the payload at that altitude. We have a flight termination system to realize that. We are currently using a linear actuator to complete this mission.

Another main requirement for us is to successfully recover the balloon. On May 03, 2026, we launched a HAB, but it ultimately failed to recover due to a loss of GPS signal. We used a commercial GPS, but it is not designed for a high-altitude object. So that is why we are going to work on our own locating and communication systems, as well as simulation and flight prediction.

### Flight Computer
For the flight computer, we design our own printed circuit board (PCB) using easyEDA, which effectively increases space utilization and reduces costs. We use the Arduino Nano as our microcontroller unit (MCU). We have a couple of sensors, including an inertial measurement unit (IMU), a pressure sensor, an SD card module, a Global Navigation Satellite System (GNSS), and a radio module. For GNSS and the radio module, we did not integrate them onto the same PCB due to signal interference concerns.

Here is the list of sensors on our board:
1. MCU: Arduino Nano ESP32/Matter
2. IMU: LSM6DSMTR
3. Pressure Sensor: BMP390
4. SD Module (Self-Built)
6. GNSS (Optional): NEO-M9N-00B

### Radio
### UI
### Simulation

## Current Progress
### Flight Computer
### Radio
### UI
### Simulation

