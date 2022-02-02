# Speeduino Compatible PCB for BMW M4x, M50 and M60 engines

This is folder for Speeduino Compatible PCBs that can be used with BMW M4x/M50/M60 engines with Bosch Motronic ecu in PnP fashion. This PCB will replace 
the original board in motronic ecu case. Just take apart the ecu case, remove original board, swap the 88-pin connector from original board to this new speeduino 
board and install back in case. Last step is to carve some openings for the extra connectors and the ecu can be plugged into car.

NOTE! This PCB supports 6-cyl sequential injection and ignition. Only sequential injection is available with arduino mega. And it requires few modifications on FW.
See "Compiling speeduino code for Arduino mega" for instructions. If you don't want to run sequential, just select semi-sequential injection in TS and you can
also use Firmware loaded by speedyloader.

NOTE 2! BMW M4x/M50/M60 engines use VR-type crank sensors and most VR-type cam sensors. So in order to run the engine with stock sensors, speeduino compatible
VR-conditioner is required. PCBs are tested to work with VR-conditioners using MAX9926 VR-conditioner. Other types might work, but it's not guaranteed. The PCBs
also allow to use Hall-type sensors. See file Jumper configurations.txt for more info.

![alt text](https://raw.githubusercontent.com/pazi88/Speeduino-M5x-PCBs/master/m50%2Cm40%2Cm60%20Pnp/Pics/20190417_081123.jpg)

Some of the features that latest rev 2.2 PCB has:
- Compatible with Speeduino FW (rev 1.x are directly compatible. Rev 2.x require small changes for 6 cyl sequential.)
- 6-cyl sequential fuel and spark capable
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
- For additional features, the pin mappings are:
   - Optional baro sensor is connected to A5
   - CEL light is connected to D26
   - Spare inputs are connected to A9, A10, A11
   - Spare relay outputs at external connectors are connected to D28, D30 and D32
   - DISA output in connected to D30 (shared with external connectod D30 output)
- PCB can be populated with HC-05/06 bluetooth module
- PCB can be populated with DIY-EFI TinyWB Module

EasyEda project link for the PCB: https://easyeda.com/pazi88/m50speeduino_copy

## Speeduino code for Arduino mega

The rev 1.x PCBs can use Speeduino FW without any changes. Just use Speeduino v0.4 board configuration in TS.
Recommended way is to use Speedyloader to upload the Firmware to mega https://wiki.speeduino.com/en/Installing_Firmware 

Rev2.0 onwards allows 6-cyl sequential injection and in order to run 6 cyl sequential 202005 or later speeduino FW is needed with few customizations.
To upload, use XLoader, instead of Speedyloader: https://www.hobbytronics.co.uk/arduino-xloader Custom FW hex can be found from here: https://github.com/pazi88/Speeduino-M5x-PCBs/tree/master/6-cyl%20firmware%20files
Remember to select ATMEGA2560 as device. Also manual compiling and upload is option. To do that, check: https://github.com/pazi88/Speeduino-M5x-PCBs/tree/master/m50%2Cm40%2Cm60%20Pnp#compiling-speeduino-code-by-yourself

NOTE! 4/8 cyl engines or semi sequential injection on 6 cyl don't require custom FW in 2.x PCBs. It's only needed for 6-cyl sequential support.
So even with 2.x PCB the speedyloder upload can be used for 4/8 cyl engines and 6 cyl semi sequential injection.

### Compiling speeduino code by yourself

If you want, you can also manually compile and upload the custom 6-cyl sequential FW. See manual compiling instructions at Wiki: https://wiki.speeduino.com/en/Installing_Firmware
Before compiling, change number of INJ_CHANNELS to 6 and number of IGN_CHANNELS to 3 in globals.h file:

![alt text](https://pazi88.kuvat.fi/kuvat/Projektikuvat/Random%20projektit/speeduino/Settings.png?img=smaller)

## DIY-EFI TinyWB Module

Rev 2.2 board has possibility to install DIY-EFI TinyWB Module internally to the ecu.
Link to the Wideband controller: https://diy-efi.co.uk/product/tinywb_module

Tiny WB uses Bosch LSU 4.9 wideband sensor. Ecu uses 6-pin Molex Minifit Jr. Connecor for the wideband sensor with following pinout:
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/LSU49_connector.png?raw=true)

