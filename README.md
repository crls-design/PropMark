# PropMark

**PropMark** is a thrust stand controller designed to benchmark the performance of heavy-lift UAV propulsion systems. This project integrates a 12S 60A ESC running the open-source ESC firmware [ESCape32](https://github.com/neoxic/ESCape32). The controller features 6 analog load cell channels for force and torque measurement, as well as USB serial communication for user interaction and live data logging, or SD card for local data logging.

The project involves both PCB design using Altium Designer and C firmware development using STM32CubeIDE.

## Features

- **Integrated 12S 60A ESC**: Capable of driving most motor-propeller combo on the market.
- **ESCape32 ESC Firmware**: Provides flexible control and telemetry.
- **6 Analog Load Cell Channels**: Supports up to 6 load cells for thrust and torque measurement.
- **USB Serial Communication**: Allows configuration, data logging, and real-time interaction with the controller.
- **Galvanic Isolation**: Ensures connected PC is safe from high power circuitry.

## Gallery

<p align="center">
    <img src="./images/v1/top wires.jpg" width="60%">
    <br>
    <img src="./images/v2/top wires.jpg" width="60%">
</p>

More in [`./images/`](./images/).

---
