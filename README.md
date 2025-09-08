
# FPV Drone PCB Full PCB + Code + Assembly

**A comprehensive, fully open-source FPV drone platform**: a two-board avionics stack (flight computer + power/ESC) complete with their codebase *and* a lightweight, robust airframe. All KiCad 9.0 design files, firmware, and CAD are included. Media of all PCBs, schematics and airframe are attached within this repository. 

## Overview

This project develops an FPV drone around two custom 5×5 PCBs: a **STM32F411 flight computer** and a **STM32F405 power distribution + ESC board**. esigned for compatibility with **Betaflight** (FC) and **AM32** (ESC). Both are built around the STM32 chipset and designed to be compatible with BetaFlight and AM32 requirements.

**Manufacturing Sponsor:** Huge thanks to **[PCBWay](https://www.pcbway.com/)** for sponsoring fabrication and assembly of the MKII flight computer. Their high-quality manufacturing, quick turnaround, significantly accelerated the build process. Moreover, their service was excellent and addressed questions promptly and clearly. As a bonus, PCBWay offers affordable, unique **silkscreen colours**. PCBWay is highly recommended and proved to be a great fit for the FPV-Drone-STM32F411 project.

**MKII Flight Computer 3D Model**

<img width="1088" height="837" alt="image" src="https://github.com/user-attachments/assets/3e55b5f0-1f1c-447c-a083-5341c130609a" />

**What’s new (MKII & Airframe)**
- **MKII Flight Computer:** UART and SWD flash options, cost-efficient and compact IMU (ICM-42605), oscilloscope-friendly SPI test pads, expanded test points, JST connectors, improved routing for debugability.
- **Open-Source Airframe:** integrated landing legs, structural reinforcements, and geometry to accommodate **~935 KV BLDC motors**; CAD provided for easy printing/mods.

## Airframe

The airframe is designed to be **robust and lightweight** with:
- **Integrated landing legs** and **structural reinforcements** for durability  
- Geometry to suit **~935 KV** brushless motors (modeled in CAD)  
- CAD provided for printing and customization

**Printing tips (suggested):**
- Arms/plates: CF-reinforced nylon or strong PLA+/PETG  
- Landing legs: TPU for impact absorption
- Verify clearances for your chosen motor + prop combo (aimed at 5″ class)


## Hardware Details

 **Integrated PCB Design**
  - Flight Controller: STM32F411CEU6 (ARM Cortex M4)
  - Power/ESC Board: High-current paths, AM32 compatible design
    
 **Connectivity**
  - All peripherals utilize SPI for high-speed sensor communication
  - USB-C, JTAG, SWD
  - UART port for external peripheral
  - Backup Coin Cell Battery Port
  - FTDI programming interface

<ins>**Flight Controller Board**</ins>
- STM32F411CEU6 microcontroller
- 12MHz external crystal oscillator
- ICM-42605 gyroscope/accelerometer
- BMP280 barometer for altitude hold
- L86-M33 GPS for postional tracking
- SX 1276 LoRa 915MHz Transciever NiceRF Module
- Power options, USB-C, LDO and External 3.3V Connection

<ins>**Power and ESC Board**</ins>

**Power System**
- SWD Flash
- DC-DC buck converter for efficient power regulation
- Support for 3S-4S LiPo batteries (11.1V-16.8V)
- XT-60 connector for battery input
- Electric Speed Controller for a BLDC motor

**Electronic Speed Controller (ESC)**
- AM32 compatible
- 10A 935 KV motor capability
- Thermal management design

## Software Compatibility

This drone design is compatible with:
- BetaFlight (recommended for racing/freestyle)
- INAV (for GPS functions)
- Ardupilot (for autonomous capabilities)
- AM32 (for ESC functionality)
- STM32CUBEIDE (Coded in C for entire codebase)

### Bill of Materials

| Component | Description | Specification |
|-----------|-------------|---------------|
| Frame | Open-Source CAD | 5″ class geometry w/ landing legs   |
| Motors | Brushless | 935 KV (example: 1800 KV 5″ also viable) |
| Battery | LiPo | 4S 1500mAh 100C (recommended) |
| Propellers | 5-inch | Matched to motor and frame |
| Camera | FPV | CCD or CMOS based on preference |
| Video Transmitter | 5.8GHz | Power output based on needs |
| Receiver | Radio control | Compatible with SX1276 LoRa Transciever |
| LoRa Antenna | 915MHz |SMA compatible quarter-wavelength antenna|


### Electronics Knowledge
- Basic soldering skills
- Understanding of power systems
- Familiarity with microcontrollers
- Ability to flash firmware through CUBEIDE

## Testing Procedures

1. Power system verification (voltage output, regulation)
2. Motor/ESC testing with oscillosocpe
3. SPI testing using oscciloscope test pads
4. Sensor calibration and verification
5. Radio range and failsafe testing
6. Controlled hover tests
7. Full flight testing

# Media
## Most Updated
### Airframe
<img width="2110" height="1390" alt="image" src="https://github.com/user-attachments/assets/0459f5f7-6173-4ced-ada2-ff094f02f9b5" />
<img width="1932" height="942" alt="image" src="https://github.com/user-attachments/assets/19bf64d2-7da5-420a-b73a-6fbba0fd5355" />


### MKII of Flight Computer 

**MKII Flight Computer 3D Model**

<img width="1088" height="837" alt="image" src="https://github.com/user-attachments/assets/3e55b5f0-1f1c-447c-a083-5341c130609a" />

**MKII Flight Computer PCB**

<img width="1116" height="1026" alt="image" src="https://github.com/user-attachments/assets/8e14f005-45c7-44f8-8ec0-955a119deb0c" />\

**MKI Flight Computer Schematic**
<img width="2002" height="1378" alt="image" src="https://github.com/user-attachments/assets/a0e9ac33-9647-4282-bd55-7698e77d00b0" />

**MKII Flight Computer Firmware Data Acquistion**

***Disclaimer: GPS outputs are example outputs***

![DATA](https://github.com/user-attachments/assets/d2ab88a2-30a0-430f-bdde-606e0899f686)

### MKI of Electric Speed Controller 

**MKI of ESC 3D Model**

<img width="911" height="907" alt="ESCV1" src="https://github.com/user-attachments/assets/b8662e06-55a8-46e5-ae89-954d1423756f" />

**MKI of ESC PCB**

<img width="790" height="846" alt="image" src="https://github.com/user-attachments/assets/320f64b6-7d53-4173-aa7f-00f5b51774b2" />

**MKI of ESC Schematic**

<img width="828" height="537" alt="image" src="https://github.com/user-attachments/assets/091e5204-9e30-41e9-a268-0c44c239c64b" />

## Historical
### MKI of Flight Computer
**MKI Flight Computer 3D Model**

![image](https://github.com/user-attachments/assets/9c4938de-f330-492c-8345-230773f7fb7f)

**MKI Flight Computer PCB**

![image](https://github.com/user-attachments/assets/25b10c26-9e5e-4a46-b159-e48ac772de75)

**MKI Flight Computer Schematic**
![image](https://github.com/user-attachments/assets/66eaace1-5f99-4fdb-9aa4-7867878309cd)


## Collaboration

This project is in collaboration with Ammar Mahmood and esb8, all PCB designs, CAD modelling, firmware modifications, and build documentation are developed jointly to ensure the highest quality and performance.

## License

This project is released under the MIT License. All design files, including schematics, PCB layouts, and firmware modifications are open source and available for personal and commercial use with attribution.

## Resources

- [PCBWay](https://www.pcbway.com/)
- [BetaFlight Documentation](https://betaflight.com/docs/getting-started/installation)
- [STM32F4 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [ESB8 GitHub Repository](https://github.com/esb8)
- [Ammar Github Repository](https://github.com/ammarjmahmood)
- [Project GitHub Repository](https://github.com/your-username/esb8-fpv-drone)

---
*This FPV drone project aims to combine and create a full documentation on the latest in drone technology with custom-designed electronics to create a high-performance, reliable platform for racing, freestyle, or aerial photography.*
