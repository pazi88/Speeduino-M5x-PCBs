# Speeduino Compatible PCB for BMW M52
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52_PnP/Pics/20200605_084533.jpg?raw=true)

This is folder for Speeduino Compatible PCBs that can be used with BMW M52 engine in PnP fashion. This PCB will replace the original board in ms41 ecu case.
Just take apart the ecu case, remove original board, swap the 88-pin connector from original board to this new speeduino board and install back in case.
Last step is to carve some openings for the extra connectors and the ecu can be plugged into car.

NOTE! This PCB supports 6-cyl sequential injection and ignition. Only sequential injection is available with arduino mega. And it requires few modifications on FW.
See "Compiling speeduino code for Arduino mega" for instructions. If you don't want to run sequential, just select semi-sequential injection in TS and you can
also use Firmware loaded by speedyloader.

NOTE 2! 6-cyl sequential is something that does not work directly in M52 with stock sensors. Instead of original m52 cam sensor, you can run M52tu / m54 intake 
cam sensor or any generic hall sensor. For that there is JP1 jumper that allows the hall sensor to be powered from 12v or 5v. If you want to put this to 12v and replace
Stock M52 Cam Sensor with M52TU / M54 intake cam sensor, it will work great. BUT DO NOT USE THAT JUMPER WITH STOCK M52 INTAKE CAM SENSOR.

![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52_PnP/Pics/20200930_102248.jpg?raw=true)

Some of the features that latest rev 2.2 PCB has:
- Compatible with Speeduino FW (rev 1.x are directly compatible. Rev 2.x require small changes.)
- 6-cyl sequential fuel in rev 2.x
- Features 16-pin external connector for following things:
    - Launch control input
    - Boost solenoid output
    - Fan relay output (also goes to engine harness)
    - Wideband lambda sensor signal input
    - Flex fuel sensor input
    - 3x spare relay output
    - 3x spare analog/digital inputs with optional bias resistors (for extra temp sensors)
    - Serial 3 Rx and Tx pins
    - +5v
    - switched + 12v for relays etc. low power devices
    - Ground
- PCB can be populated with optional BARO sensor
- PCB can be populated with optional 2nd pressure sensor. For example for exhaust back pressure sensing
- VSS input through the stock wiring
- Possibility to use stock lambda sensors (untested feature) or use those as extra analog inputs.
- For additional features, the pin mappings are:
   - Optional baro sensor is connected to A5
   - CEL light is connected to D37
   - Spare inputs are connected to A6, A7, A9
   - Spare relay outputs at external connectors are connected to D31, D33 and D35
- PCB can be populated with HC-05/06 bluetooth module
- PCB can be populated with DIY-EFI TinyWB Module

EasyEda project link for the PCB: https://easyeda.com/pazi88/speeduino-v0-4-3-compatible-pcb-for-m52-rev1-2_copy

## Speeduino code for Arduino mega

The rev 1.x PCBs can use Speeduino FW without any changes. Just use Speeduino v0.4 board configuration in TS.
Recommended way is to use SpeedyLoader to upload the Firmware to mega https://wiki.speeduino.com/en/Installing_Firmware

SpeedyLoader download: https://speeduino.com/home/support/downloads

Rev2.0 onwards allows 6-cyl sequential injection and in order to run 6 cyl sequential, custom FW is needed. This can be also uploaded with SpeedyLoader, but minimum
of 1.6.0 version is required. First download pre-compiled custom FW from here: https://github.com/pazi88/Speeduino-M5x-PCBs/tree/master/6-cyl%20firmware%20files
Then on SpeedyLoader select "Use Local Firmware"
![alt text](https://github.com/pazi88/STM32_mega/blob/main/Pics/Speedyloader1.png?raw=true)

And upload the FW through correct COM-port.

### Compiling speeduino code by yourself

If you want, you can also manually compile and upload the custom 6-cyl sequential FW. See manual compiling instructions at Wiki: https://wiki.speeduino.com/en/Installing_Firmware
Before compiling, change number of INJ_CHANNELS to 6 and number of IGN_CHANNELS to 3 in globals.h file:

![alt text](https://pazi88.kuvat.fi/kuvat/Projektikuvat/Random%20projektit/speeduino/Settings.png?img=smaller)

## ECU rear connector pinout

![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/tree/master/m52_PnP/Pics/20.png?raw=true)

![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/tree/master/m52_PnP/Pics/20-21.png?raw=true)

![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/tree/master/m52_PnP/Pics/30.png?raw=true)

## DIY-EFI TinyWB Module

Rev 2.2 board has possibility to install DIY-EFI TinyWB Module internally to the ecu.
Link to the Wideband controller: https://diy-efi.co.uk/product/tinywb_module

Tiny WB uses Bosch LSU 4.9 wideband sensor. Ecu uses 6-pin Molex Minifit Jr. Connecor for the wideband sensor with following pinout:
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu-m54_PnP/Pics/LSU49_connector.png?raw=true)

