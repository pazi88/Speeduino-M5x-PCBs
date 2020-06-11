# Speeduino Compatible PCBs for BMW M52TU and M54 engines
![alt text] (https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/rev12.jpg)

This is folder for Speeduino Compatible PCBs that can be used with BMW M52TU and M54 engine in PnP fashion. This PCB will replace the
original board in ms42/ms43 ecu case. Just open the 4 bolts in the ecu case, remove original board and replace with this. Last step is
to carve some openings for the extra connectors and the ecu can be plugged into car.

![alt text] (https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/withcase.jpg)

Some of the features this PCB has:
- Compatible with Speeduino FW (rev1.0/1.1 are directly compatible. Rev1.2/1.3 require small changes.)
- 6-cyl sequential fuel in rev1.2/1.3
- Uses DIY-EFI Core4 ECU module, instead of Arduino Mega. For more info: https://diy-efi.co.uk/product/core4-module
- Provision to use Spartan2 OEM wideband controller. For more info: https://www.14point7.com/products/spartan-2-oem
- Provision to use DIY-EFI TinyWB wideband controller. (rev1.3)
- Includes CAN-bus interface to control original instrument cluster in bmw e46/e38/e39 chassis. For other cars with regular cluster, this isn't needed.

## Compiling speeduino code for Core4

The rev1.0/1.1 PCBs can use Speeduino FW without any changes. Just use 201909 speeduino FW release or newer and select DIY-EFI CORE4 v1.0 as board layout in TS.
Recommended way is to use Speedyloader to upload the Firmware to CORE4: https://wiki.speeduino.com/en/Installing_Firmware Rev1.2 onwards allows 6-cyl sequential 
injection and in order to run those 202005 or later speeduino FW is needed. Also the easy Speedyloader FW upload can't be used but it requires manual compiling
with small changes in code. See manual compiling instructions in Wiki at earlier link. Before compiling, change number of INJ-CHANNELS to 6 and number of IGN-CHANNELS
to 3 in globals.h file:
![alt text](https://pazi88.kuvat.fi/kuvat/Projektikuvat/Random%20projektit/speeduino/Settings.png?img=smaller)

It's also recommended to change number of fuel outputs to 6 instead of 4 in speeduino.ini -file for Core4. But this isn't mandatory. It just gets rid of the possible
warnings in TS and allows to use HW test mode for injectors 5 and 6.

## Compiling code for CAN-Bus interface

The CAN-bus interface uses STM32F103C8T6 bluepill board in conjunction of MCP2551 CAN transceiver to send CAN data to instrument cluster and read
data from speeduino using Serial3. In order to program the bluepill for CAN-interface, you will need FTDI breakout board and the code is meant to be 
used in Arduino IDE. The code works with regular STM32 core board manager for Arduino: https://github.com/stm32duino/Arduino_Core_STM32

Use these settings in Arduino IDE to compile and upload the code. Choose correct COM port for your FTDI breakout board:
![alt text](https://raw.githubusercontent.com/pazi88/8Ch-EGT/master/Arduino%20IDE%20settings.png)

NOTE! Use the Serial3toBMWcan_BluePill -code with the bluepill. The other codes are to be used rev1.0/1.1 that used different CAN-bus interface. But
due to problems in those, the older CAN-bus interface is not recommended to be used.


EasyEda project link for the PCB: https://easyeda.com/pazi88/ms42-43-compatible-speeduino-PnP-Core4
