
# FPV Drone PCB Full PCB + Assembly - Work in Progress

A comprehensive open-source avionics ecosystem for FPV drones featuring a completed flight computer PCB along with in-development power distribution and electronic speed controller PCB. This project delivers an avionics stack for a custom drone with full schematic and PCB files available on KiCad 9.0.

## Overview

This project aims to create a FPV drone with a custom all-in-one (AIO) PCB that combines the flight controller, power distribution, and ESC functionality. Built around the STM32F411CEU6 microcontroller, this drone follows BetaFlight and AM32 design requirements while providing a clean and efficient build.

## Hardware 

 **Integrated PCB Design**
  - Flight Controller: STM32F411CEU6 (ARM Cortex M4)
  - Power Distribution: High-current paths with filtering
  - ESC: AM32 compatible 4-in-1 design
    
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
- Electric Speed Controller for four BLDC motors

**ESC (Electronic Speed Controller)**
- AM32 compatible
- 30A motor capability
- Thermal management design
 

## Software Compatibility

This drone design is compatible with:
- BetaFlight (recommended for racing/freestyle)
- INAV (for GPS functions)
- Ardupilot (for autonomous capabilities)
- AM32 (for ESC functionality)


### Bill of Materials

| Component | Description | Specification |
|-----------|-------------|---------------|
| Frame | 3D printed carbon fiber | 5" freestyle/racing frame |
| Motors | Brushless | SpeedyBee 1800KV (recommended) |
| Battery | LiPo | 4S 1500mAh 100C (recommended) |
| Propellers | 5-inch | Matched to motor and frame |
| Camera | FPV | CCD or CMOS based on preference |
| Video Transmitter | 5.8GHz | Power output based on needs |
| Receiver | Radio control | Compatible with SX1276 LoRa Transciever |
| LoRa Antenna | 915MHz |SMA compatible half-dipole antenna|

### Tools
- Soldering iron and solder
- Heat shrink tubing
- Hex drivers/screwdrivers
- LiPo battery charger
- Multimeter
- KiCAD for PCB design modifications
- Zip ties
- Double-sided foam tape

### Electronics Knowledge
- Basic soldering skills
- Understanding of power systems
- Familiarity with microcontrollers
- Ability to flash firmware

## Calculations

### Power System
- Max Current = 4 motors × 30A = 120A
- Required C Rating = 120A / 1.5Ah = 80C minimum
- Flight time estimate = (1500mAh) ÷ (Average current draw) × 60 × 0.8

### Weight Budget
- Target all-up-weight: ~500g (5" freestyle quad)
- Battery weight: ~120-150g (4S 1500mAh)
- Component weight distribution accordingly

## Testing Procedures

1. Power system verification (voltage output, regulation)
2. Motor/ESC testing with oscillosocpe
3. Sensor calibration and verification
4. Radio range and failsafe testing
5. Controlled hover tests
6. Full flight testing

# Media
### MKI of Electric Speed Controller (ESC)

**MKI of ESC 3D Model**

<img width="885" height="836" alt="image" src="https://github.com/user-attachments/assets/f082285e-2f15-4693-bd55-5e7dd71b4ff9" />

**MKI of ESC PCB**

<img width="790" height="846" alt="image" src="https://github.com/user-attachments/assets/320f64b6-7d53-4173-aa7f-00f5b51774b2" />

**MKI of ESC Schematic**

<img width="828" height="537" alt="image" src="https://github.com/user-attachments/assets/091e5204-9e30-41e9-a268-0c44c239c64b" />


### MKII of Flight Computer (Most Updated)

**MKII Flight Computer 3D Model**


<img width="1585" height="1537" alt="image" src="https://github.com/user-attachments/assets/4a1bf0b9-8b31-4664-99bb-4b8587f5d55a" />





**MKII Flight Computer PCB**

<img width="1116" height="1026" alt="image" src="https://github.com/user-attachments/assets/8e14f005-45c7-44f8-8ec0-955a119deb0c" />


### MKII of Flight Computer
**MKI Flight Computer Schematic**
<img width="2002" height="1378" alt="image" src="https://github.com/user-attachments/assets/a0e9ac33-9647-4282-bd55-7698e77d00b0" />

**MKI Flight Computer 3D Model**

![image](https://github.com/user-attachments/assets/9c4938de-f330-492c-8345-230773f7fb7f)

**MKI Flight Computer PCB**

![image](https://github.com/user-attachments/assets/25b10c26-9e5e-4a46-b159-e48ac772de75)

**MKI Flight Computer Schematic**
![image](https://github.com/user-attachments/assets/66eaace1-5f99-4fdb-9aa4-7867878309cd)






## Collaboration

This project is in collaboration with Ammar Mahmood and esb8, All PCB designs, firmware modifications, and build documentation are developed jointly to ensure the highest quality and performance.

## License

This project is released under the MIT License. All design files, including schematics, PCB layouts, and firmware modifications are open source and available for personal and commercial use with attribution.

## Resources

- [BetaFlight Documentation](https://betaflight.com/docs/getting-started/installation)
- [STM32F4 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [ESB8 GitHub Repository](https://github.com/esb8)
- [Ammar Github Repository](https://github.com/ammarjmahmood)
- [Project GitHub Repository](https://github.com/your-username/esb8-fpv-drone)

## Contributing

We welcome contributions to this project! If you have suggestions, bug reports, or want to contribute code, please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request

---
*This FPV drone project aims to combine and create a full documentation on the latest in drone technology with custom-designed electronics to create a high-performance, reliable platform for racing, freestyle, or aerial photography.*
