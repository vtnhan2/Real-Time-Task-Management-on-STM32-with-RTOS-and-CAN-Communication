# Embedded Systems Project - Group 13

## Project Overview
This project implements an embedded system with multiple functionalities including:
- LCD display interface with touch screen control
- CAN bus communication
- Temperature monitoring
- DAC-based tone generation
- LED control
- Sleep mode functionality

## Hardware Requirements
- STM32 microcontroller (specific model not mentioned)
- LCD display with touch interface (ST7789 controller)
- CAN bus transceivers (for CAN1 and CAN2)
- LED for status indication
- DAC output for audio generation
- UART interface for debugging

## Key Features

### Display System
- Two main screens:
  - Base screen with project information and team members
  - Main menu with four task zones
- Touch screen navigation:
  - "NEXT" button to move from base screen to main menu
  - "BACK" button to return to base screen
  - Four task zones for activating different functionalities

### Task Functionalities
1. **Task 02-1 (LED Control)**
   - Toggles an LED at 1Hz when activated
   - Turns off LED when deactivated

2. **Task 02-2 (CAN Communication)**
   - Sends temperature data via CAN1 every 500ms when activated
   - Data format: Group ID (1 byte) + Temperature (4-byte float)

3. **Task 02-3 (CAN Display)**
   - Displays received CAN messages (Group ID and Temperature)
   - Shows data for 3 seconds then returns to main menu
   - Listens on CAN2 for messages with ID 0x321

4. **Task 02-4 (DAC Alarm)**
   - Plays a 10-second alarm tone sequence when activated
   - Enters sleep mode after alarm completes
   - Wakes up on external interrupt

### Additional Features
- Touch screen calibration and debugging
- Internal temperature sensor reading
- UART debug output
- Power-efficient sleep mode

## Software Architecture
The project uses FreeRTOS with multiple tasks:
1. `Task01_Display`: Handles LCD display updates
2. `Task02_Touch`: Processes touch screen input
3. `Task_LED`: Controls LED blinking
4. `Task_CAN`: Manages CAN communication
5. `Task_DispCAN`: Displays received CAN messages
6. `Task_DAC`: Generates tones and manages sleep mode

## Configuration
- System clock: 16MHz HSI with PLL to 256MHz, divided to 128MHz system clock
- CAN baud rate: 1Mbps (prescaler 16)
- UART baud rate: 9600
- LCD rotation: Landscape mode (2)

## Usage
1. Power on the system to see the base screen
2. Touch "NEXT" to access the main menu
3. Touch any task zone to activate its functionality:
   - Task zones will highlight in green when active
   - Some tasks automatically return to menu after completion
4. Touch "BACK" to return to the base screen

## Debugging
Debug information is sent via UART4 (9600 baud) including:
- Touch screen coordinates
- Task activation/deactivation status
- CAN message details
- Temperature readings
- System state changes

## License
This project is licensed under the terms of the STMicroelectronics license as specified in the source code headers.
