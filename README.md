# STM32 Attitude and Heading Reference System (AHRS)

A standalone Attitude and Heading Reference System built on STM32, performing real-time sensor fusion of IMU data to estimate orientation (roll, pitch, yaw) without reliance on external positioning aids.

## Overview

This project implements an embedded AHRS suitable for applications requiring reliable, drift-corrected attitude estimation — e.g. UAV/avionics platforms, robotics, and stabilization systems. The system reads raw inertial data, applies calibration to remove sensor bias and distortion, and fuses accelerometer, gyroscope, and magnetometer measurements into a stable orientation estimate.

## Hardware

- **MCU:** STM32 [series/part number]
- **IMU:** [sensor part number — accelerometer/gyroscope]
- **Magnetometer:** [sensor part number]
- **Interface:** SPI
- **Other:** [power regulation, connectors, enclosure, etc.]

## Features

- Real-time sensor fusion (e.g. Madgwick / Mahony / Kalman filter — *specify which*)
- Hard-iron and soft-iron magnetometer calibration
- Gyroscope bias estimation via Allan deviation analysis
- SPI-based sensor interfacing at [update rate] Hz
- [Output interface — UART/USB telemetry, logging, etc.]

## System Architecture

```
[IMU + Magnetometer] --SPI--> [STM32: Sensor Fusion Filter] --> [Orientation Output]
```

[Optional: link to KiCAD schematic / block diagram here]

## Calibration

- **Magnetometer:** Hard-iron and soft-iron calibration performed via [ellipsoid fitting / least-squares method] to correct for fixed and induced magnetic distortion.
- **Gyroscope:** Bias and noise characteristics quantified using Allan deviation analysis to determine sensor noise parameters (angle random walk, bias instability) used to tune the fusion filter.

## Results

[Plots/metrics: e.g. static orientation error, drift over time, Allan deviation curve, before/after calibration comparison]

## Repository Structure

```
/Core         - Main application and fusion algorithm
/Drivers      - Sensor drivers (SPI IMU/magnetometer interfacing)
/Calibration  - Magnetometer calibration and Allan deviation scripts
/Hardware     - KiCAD schematic and PCB files
```

## Build & Flash

Built using STM32CubeIDE.

1. Clone the repository
2. Open the project in STM32CubeIDE
3. Build and flash to target hardware via ST-Link

## Future Work

- [e.g. EKF-based fusion, GPS integration, flight-test validation]

## Author

Matthew Windsor — [LinkedIn/portfolio link]
