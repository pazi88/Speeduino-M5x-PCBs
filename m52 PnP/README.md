# Speeduino Compatible PCB for BMW M52
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52%20PnP/Pics/20200605_084533.jpg?raw=true)

This is folder for Speeduino Compatible PCBs that can be used with BMW M52 engine in PnP fashion. This PCB will replace the original board in ms41 ecu case.
Just take apart the ecu case, remove original board, swap the 88-pin connector from original board to this new speeduino board and install back in case.
Last step is to carve some openings for the extra connectors and the ecu can be plugged into car.

NOTE! This PCB supports 6-cyl sequential injection and ignition. Only sequential injection is available with arduino mega. And it requires few modifications on FW.
See "Compiling speeduino code for Arduino mega" for instructions. If you don't want to run sequential, just select semi-sequential injection in TS and you can
also use Firmware loaded by speedyloader.

NOTE 2! 6-cyl sequential is something that does not work directly in M52 with stock sensors. Instead of original m52 cam sensor, you can run M52tu / m54 intake 
cam sensor or any generic hall sensor. For that there is JP1 jumper that allows the hall sensor to be powered from 12v or 5v. If you want to put this to 12v and replace
Stock M52 Cam Sensor with M52TU / M54 intake cam sensor, it will work great. BUT DO NOT USE THAT JUMPER WITH STOCK M52 INTAKE CAM SENSOR.

![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52%20PnP/Pics/20200930_102248.jpg?raw=true)

Some of the features this rev 2.1 PCB has:
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

## Compiling speeduino code for Arduino mega

The rev 1.x PCBs can use Speeduino FW without any changes. Just use Speeduino v0.4 board configuration in TS.
Recommended way is to use Speedyloader to upload the Firmware to mega https://wiki.speeduino.com/en/Installing_Firmware 

Rev2.0 onwards allows 6-cyl sequential injection and in order to run sequential 202005 or later speeduino FW is needed. Also the easy Speedyloader 
FW upload can't be used but it requires manual compiling with small changes in code. See manual compiling instructions in Wiki at earlier link. 
Before compiling, change number of INJ_CHANNELS to 6 and number of IGN_CHANNELS to 3 in globals.h file:

![alt text](https://pazi88.kuvat.fi/kuvat/Projektikuvat/Random%20projektit/speeduino/Settings.png?img=smaller)

EasyEda project link for the PCB: https://easyeda.com/pazi88/speeduino-v0-4-3-compatible-pcb-for-m52-rev1-2_copy
