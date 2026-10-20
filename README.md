# Project-GeoDrop

Project GeoDrop is a senior design project focused on developing an asset tracking and drop detection system for medical equipment.

The project is being developed in collaboration with Becton Dickinson (BD). The goal is to investigate technologies that could be used to locate medical devices, such as infusion pumps, while also detecting potential drop events.

## Current Development

The current phase of the project focuses on evaluating Bluetooth Channel Sounding for distance measurement.

Using Texas Instruments development hardware, the current prototype is able to:

- Establish a Bluetooth Low Energy (BLE) connection between two devices
- Perform Bluetooth Channel Sounding measurements
- Obtain real-time distance estimates
- Obtain confidence values associated with each measurement
- Visualize distance and confidence measurements in real time using Python and Matplotlib

## Hardware

Current development hardware includes:

- Texas Instruments CC2755P10 development hardware
- Texas Instruments CC2745R10-CS-EVM
- XDS110 debug/serial interface

The two TI radio devices perform Bluetooth Channel Sounding wirelessly. The development computer communicates with the system through the XDS110 interface.

## Software

Current software development uses:

- Python
- Matplotlib
- Texas Instruments SimpleLink Low Power F3 SDK 9.23.00.27
- TI BLE Agent

The Channel Sounding implementation in this repository is based on a Texas Instruments BLE Agent example and has been modified for the GeoDrop project.

The TI SimpleLink SDK is not included in this repository and must be installed separately.

## Repository Structure

```text
Project-GeoDrop/
├── README.md
├── LICENSE-TI.txt
└── channel_sounding/
    └── ble_device_car_node_live_graph.py
