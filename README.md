# EV Battery Charger Controller

## Overview
This repository contains the firmware and hardware schematics for a microcontroller-based EV battery charger. The system features adjustable Constant Current (CC) and Constant Voltage (CV) outputs, active thermal protection, and a 4-digit 7-segment user interface. It utilizes a PI control loop to regulate power and includes a multi-stage battery detection system.

## Features
* **Adjustable Charging Profiles**: Supports multiple VSET modes ranging from 54.6V to 87.6V. It also supports CSET modes ranging from 6.2A to 10.2A. The default configuration is 67.2V at 6.2A.
* **Intelligent Charging Phases**: Features a pre-charge mode at 0.5A for deeply depleted batteries. It includes a deep discharge charging mode at 1.0A. It executes a normal CC-CV soft-start ramp of 0.3A/sec.
* **Advanced Protection**: Implements Short Circuit Protection (SCPT) triggering at 150% of CSET. It includes AC Over/Under Voltage Protection (OVPT/UNVP). It features a 7-hour total charge timeout (CHTO). 
* **Thermal Management**: Monitors MOSFET and transformer temperatures using 10k NTC thermistors (MF11-103). It automatically derates the charge current when temperatures reach 80°C. It triggers a complete shutdown fault (HITP) at 90°C.
* **User Interface**: Uses a TM1637 7-segment display to alternate between showing the battery voltage and the State of Charge (SOC). It also displays specific fault codes like "BTNG" (Battery Not Good) or "DPDC" (Deep Discharge).

## Hardware Architecture
* **Microcontroller**: PY32F002AF15P6TU (32-bit MCU).
* **PWM Controller**: TL494CN.
* **Gate Driver**: UCC27714D.
* **Sensing**: Utilizes a PC817 optocoupler for UV/OV sensing and dedicated ADC channels for output voltage and current feedback.

## Repository Structure
* `/Firmware`: Contains the C source code (`main.c`, `main.h`, peripheral drivers) for the MCU.
* `/Hardware`: Contains the system schematic (`EV Battery charge Update Sch.pdf`).

**Author**: Gautam