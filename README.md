# UWB Full-Stack Localization System

A comprehensive ultra-wideband (UWB) localization framework combining custom hardware design and full-stack firmware and software implementation for precise indoor positioning. The system leverages Qorvo DW3000 devices, ESP32 microcontrollers for anchors, a DWM3001CDK module for the tag, and a Raspberry Pi 4 gateway. It achieves centimeter-level positioning accuracy, with a static mean error as low as **1.9 cm** under optimal LOS conditions and **~10 cm dynamic accuracy** at normal walking speed in office environments.

> This work is described in the paper: **"High-Precision UWB Novel Localization Framework for Proximity-Based Interaction in Smart Spaces"** - László Benedek & Mircea Giurgiu, Telecommunications Department, Technical University of Cluj-Napoca, Romania.

## Overview

> **Note**: This repository currently contains the anchor hardware design and parts of the low-level firmware implementation. Additional components including tag hardware, gateway software, backend systems, and positioning algorithms might be published in future releases or upon request.

This project presents a novel hardware and software framework for high-precision indoor localization using UWB technology. The system integrates:

- **Modular Hardware**: Three fixed anchor nodes built around ESP32 MCU + Qorvo DWM3000EVB shields, one mobile tag using the Qorvo DWM3001CDK (DW3110 transceiver + BLE SoC), and a Raspberry Pi 4 as the gateway.
- **Distance Ranging**: Double-Sided Two-Way Ranging (DS-TWR) using a three-message sequence (Poll → Response → Final) to compute Time of Flight (ToF) without requiring strict clock synchronization between nodes.
- **Position Estimation Pipeline**: Levenberg-Marquardt (LM) algorithm for non-linear 2D position approximation, combined with a recursive Kalman filter applied to raw distance measurements for dynamic noise mitigation.
- **Anchor Self-Localization**: Autonomous initialization using the law of cosines - anchors determine their spatial configuration without manual coordinate entry.
- **Communication Stack**: Custom binary protocol encoded with Protocol Buffers, encapsulated in IEEE 802.15.4 MAC frames within a Personal Area Network (PAN). Data is forwarded to the gateway as JSON over serial, then distributed via an MQTT broker.
- **Data Processing**: Node-RED flow-based architecture on the Raspberry Pi 4, hosting Kalman filtering, LM positioning, virtual marker activation (hysteresis-based), and MQTT publishing.
- **Digital Twin Interface**: React-based Progressive Web App (PWA) integrating Smplrspace (2D/3D digital twin of the physical space) and Desmos (developer-centric coordinate validation with interactive sliders and draggable markers).
- **Smart Device Integration**: A headset or speaker outputs location-specific audio based on the user's proximity to predefined virtual markers in the space.

### Key Performance Results

| Condition                                  | Mean Error       | Max Deviation |
| ------------------------------------------ | ---------------- | ------------- |
| Static – center of anchor triangle (LOS)   | 1.9 cm           | 6.4 cm        |
| Static – LOS outside anchor triangle       | 22.9 cm          | 24.2 cm       |
| Static – highly reflective NLOS            | 46.8 cm          | 68.9 cm       |
| Static – outside office (strong NLOS)      | 66.3 cm          | 108.3 cm      |
| Dynamic – normal walking pace (9.6 m path) | ~13 cm (lateral) | ~49 cm        |
| Dynamic – fast walking pace                | ~34 cm (lateral) | ~69 cm        |

- **Marker activation success rate**: 100% at normal walking speed, 80% at fast walking speed.
- **Anchor self-localization**: mean Euclidean error of 2.4 cm for A2 and 5.4 cm for A3.

### Competitive Comparison

| System                           | Anchors | Typical Accuracy                |
| -------------------------------- | ------- | ------------------------------- |
| SafeLocSystems (commercial)      | 6       | 21 cm                           |
| DS-TWR with residual weighting   | 4       | 10 cm                           |
| RedPoint Healthcare (commercial) | -       | 10 cm                           |
| DWM1000, TDOA (retail)           | 4       | 15 cm                           |
| DWM1001C, TDOA                   | 4       | 30 cm                           |
| **This system**                  | **3**   | **2 cm static / 10 cm dynamic** |

## System Architecture

The framework consists of four primary entities:

1. **Tag** - Mobile DWM3001CDK node (DW3110 UWB transceiver + BLE SoC), initiates DS-TWR ranging with all anchors.
2. **Anchors** - Three fixed ESP32 + DWM3000EVB nodes. Each responds to the tag's ranging requests and calculates distance. **Anchor A acts as coordinator**, aggregating data from Anchors B and C and forwarding it to the gateway via serial/USB.
3. **Gateway** - Raspberry Pi 4. Receives aggregated timestamped distances from Anchor A, runs position estimation (LM + Kalman) in Node-RED, hosts the MQTT broker, and serves the web frontend.
4. **Smart Device** - Headset or speaker triggered by proximity-based events (e.g., audio playback when the tag enters a virtual marker zone).

## Quick Start

### Prerequisites

- **Hardware** (choose one configuration):
  - **DWM3001CDK**: All-in-one module with integrated nRF52 host MCU and DW3000 UWB transceiver - recommended for the tag.
  - **ESP32 + DWM3000EVB**: ESP32 development board paired with Qorvo DWM3000EVB evaluation board - used for anchor nodes.
  - 18650 Li-ion battery + TP4056 power management module (for autonomous anchor operation)
  - USB cable for programming
- **Software**:
  - [Visual Studio Code](https://code.visualstudio.com/)
  - [PlatformIO IDE Extension](https://platformio.org/) - for building and uploading firmware
  - Qorvo DW3000 API Version 08.02.02
  - [Python 3.7+](https://www.python.org/) - for utility scripts
  - Git

### Installation

1. **Clone the repository**

   ```bash
   git clone https://github.com/smilexs4/UWB-FullStack-Localization.git
   cd UWB-FullStack-Localization
   ```

2. **Install PlatformIO IDE Extension** in VS Code
   - Open VS Code → Extensions (Ctrl+Shift+X / Cmd+Shift+X)
   - Search for "PlatformIO IDE" → Click "Install"

3. **Open the project in VS Code**

   ```bash
   code UWB-FullStack-Localization
   ```

4. **Navigate to the firmware directory**

   ```bash
   cd uwb-positioning-sensors
   ```

5. **Configure your board** in `platformio.ini` if needed for your specific target (ESP32 or nRF52).

### Building and Uploading

All build and upload operations are performed through the PlatformIO IDE VS Code extension:

- **Build**: Click the PlatformIO home icon → select your project → click "Build"
- **Upload**: Connect your device via USB → click "Upload" in PlatformIO project tasks
- **Monitor**: Click "Serial Monitor" in PlatformIO tasks to view real-time output

### Batch Flashing Multiple Devices

To flash multiple anchor devices simultaneously, configure `platformio.ini`:

```ini
extra_scripts = multi_esptool.py
simultaneous_upload_ports = COM2, COM3
```

Replace `COM2, COM3` with the actual COM ports of your devices, then click "Upload".

## Project Structure

```
uwb-anchor-design-hardware/     # Hardware PCB designs and schematics
└── EasyEDA design files        # Designed with EasyEDA, manufactured externally

uwb-positioning-sensors/         # Firmware implementation
├── lib/                          # Libraries and drivers
│   ├── dw3000/                   # Qorvo DW3000 definitions
│   ├── dwt_uwb_driver/           # UWB driver
│   ├── qmath/                    # Math utilities
│   ├── port/                     # Platform-specific code (ESP32, nRF52)
│   └── util/                     # Utility functions
├── src/                          # Source code
├── test/                         # Unit tests
├── platformio.ini                # PlatformIO build configuration
└── multi_esptool.py              # Multi-device flashing utility
```

## Hardware

### Anchor Node: ESP32 + DWM3000EVB

- **Microcontroller**: ESP32
- **UWB Module**: Qorvo DWM3000EVB (contains DW3000 series UWB transceiver)
- **Power**: TP4056 power management module with 18650 Li-ion battery (~12 hours active at 50 measurements/sec, ~15 hours passive, from a 2000 mAh cell)
- **PCB**: Designed in EasyEDA, manufactured by a private company, manually assembled and soldered. Uses female headers for plug-and-play insertion of development boards.
- **Three anchors** are deployed: Anchor A (coordinator), Anchor B, and Anchor C. Anchors were deployed at ~80 cm height during experiments.

### Tag Node: DWM3001CDK

- **Module**: Qorvo DWM3001CDK - integrates a DW3110 UWB transceiver with a Bluetooth-enabled SoC (nRF52)
- **Power**: ~100 hours active (50 measurements/sec) from a 2000 mAh battery; ~200 hours in low-power standby (1 re-attempt/sec)
- **Features**: All-in-one solution ideal for rapid prototyping.

See [uwb-anchor-design-hardware/README.md](uwb-anchor-design-hardware/README.md) for detailed PCB designs and assembly information.

## Firmware

The firmware is built using PlatformIO targeting the Arduino framework, providing a consistent codebase across ESP32 and nRF52 platforms. It integrates Qorvo DW3000 API Version 08.02.02 and provides:

- Low-level UWB transceiver control
- IEEE 802.15.4 MAC frame handling (POLL, RESP, FINAL messages encapsulated in short-address frames within a PAN)
- Custom binary protocol encoded with Protocol Buffers for compact message serialization
- **Interrupt-enabled driver**: maps the DW3000 hardware interrupt pin to the host MCU's GPIO, allowing the MCU to sleep and wake only on critical radio events (frame reception/transmission). This avoids continuous register polling, reduces power consumption, and enables high ranging frequencies with low-latency response.
- DS-TWR ranging logic implementing the three-message exchange and ToF calculation

A custom Wireshark sniffer tool is also provided for real-time MAC-layer debugging of UWB frame exchanges.

See [uwb-positioning-sensors/README.md](uwb-positioning-sensors/README.md) for detailed firmware documentation.

## Algorithms

### DS-TWR (Double-Sided Two-Way Ranging)

Distance is computed from a three-message exchange (Poll → Response → Final). The ToF is derived as:

```
ToF = (T_round1 × T_round2 − T_reply1 × T_reply2) / (T_round1 + T_round2 + T_reply1 + T_reply2)
```

This eliminates the need for tight clock synchronization between anchors.

### Position Estimation: Levenberg-Marquardt Algorithm

Given three anchor positions and their corresponding measured distances to the tag, the LM algorithm minimizes the non-linear least-squares cost function to estimate the 2D tag position. The algorithm iteratively updates the position estimate using the Jacobian of residuals and a damping factor λ.

### Distance Filtering: Kalman Filter

A recursive Kalman filter is applied to raw distance measurements before position estimation. It uses a linear motion model for the prediction phase and corrects estimates based on new measurements, minimizing RMSE of final position estimates.

### Anchor Self-Localization

Anchors autonomously determine their relative positions using the law of cosines on inter-anchor distances. Assuming A = (0, 0) and B = (d_AB, 0), the position of anchor C is derived without any manual coordinate entry. Experimental results: mean error 2.4 cm for A2, 5.4 cm for A3.

## Citation

<!-- If you use this work, please cite:

> L. Benedek and M. Giurgiu, "High-Precision UWB Novel Localization Framework for Proximity-Based Interaction in Smart Spaces," AQTR 2026, Technical University of Cluj-Napoca, Romania. -->

This work is supported by the project **"Romanian Hub for Artificial Intelligence – HRIA"**, MySMIS no. 351416.

⚠️ **Link to the paper will become active after publication.**
