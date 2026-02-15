# UWB Positioning Sensors

> **Warning**
> This repository is only a portion of the complete project, which is mostly private. As a result, it may not compile without issues or on every specified platform. If you run into problems, feel free to open a ticket.

This repository contains the low-level PlatformIO implementation of the UWB Localization Framework. It has been tested on ESP32 and nRF52 platforms.

## Resources

This implementation uses the **DW3_QM33_SDK_1.0.2** release, available on the [Qorvo Forum](https://forum.qorvo.com/t/new-release-of-dw3000-and-qm33-sdk-dw3-qm33-sdk-v1-0-2/19160/1).

Makerfabs reference materials are available at: https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000

## Directory Structure

- **/lib/util**
  - **dw3000_mac_802_15_4_short**: Modified, stripped-down Makerfabs IEEE 802.15.4 frame format for 16-bit PAN short addressing.
  - **util_uwb**: Utility functions for interrupt-enabled interaction with Qorvo's library.
  - **util**: Generic utility functions.

- **/lib/port**
  - **dw3000_port**: Makerfabs port for the ESP32.
  - **dw3000_port_mod**: Additional auxiliary driver features.

- **/LICENSES**: Copied from `DW3_QM33_SDK_1.0.2/Drivers/API/Shared/dwt_uwb_driver/LICENSES`.

- **/lib/dw3000**: Copied from `DW3_QM33_SDK_1.0.2/Drivers/API/Shared/dwt_uwb_driver/dw3000`.

- **/lib/dwt_uwb_driver**: Copied from `DW3_QM33_SDK_1.0.2/Drivers/API/Shared/dwt_uwb_driver/*`.

- **/lib/qmath**: Copied from `DW3_QM33_SDK_1.0.2/Drivers/API/Shared/dwt_uwb_driver/lib/qmath`.

## License

Original code in this repository, primarily identified by `@author smilexs4` and `Copyright (c) 2025`, is licensed under the [MIT License](LICENSE.md).

Third-party components, including those copied from `DW3_QM33_SDK_1.0.2` (e.g., in `lib/dw3000`, `lib/dwt_uwb_driver`, `lib/qmath`), retain their original licensing as specified in `LICENSES/LicenseRef-QORVO-2.txt` and their respective files.
