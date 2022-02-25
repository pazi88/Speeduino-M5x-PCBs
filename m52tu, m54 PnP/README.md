# Speeduino Compatible PCBs for BMW M52TU and M54 engines
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/rev12.jpg?raw=true)

This is folder for Speeduino Compatible PCBs that can be used with BMW M52TU and M54 engine in PnP fashion. This PCB will replace the
original board in ms42/ms43 ecu case. Just open the 4 bolts in the ecu case, remove original board and replace with this. Last step is
to carve some openings for the extra connectors and the ecu can be plugged into car.

NOTE! DBW is not supported, so in M54 engine (ms43), you need to change to cable driven throttle body in order to use this.
M52TU (ms42) has cable driven throttle as backup, so no changes needed in those to use this speeduino board. See "Replacing original DBW throttle body" -chapter

NOTE 2! OBD2 system is not supported in these boards, so if your country requires the car to pass OBD2 tests in MOT check, this board can't be used.

![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/withcase.jpg?raw=true)

Some of the features this PCB has:
- Compatible with Speeduino FW (rev1.0/1.1 are directly compatible. Rev1.2/1.3 require small changes.)
- 6-cyl sequential fuel in rev1.2 and later
- Uses DIY-EFI Core4 ECU module, instead of Arduino Mega. For more info: https://diy-efi.co.uk/product/core4-module
- Provision to use Spartan2 OEM wideband controller. For more info: https://www.14point7.com/products/spartan-2-oem
- Provision to use DIY-EFI TinyWB wideband controller. (rev1.3)
- Includes CAN-bus interface to control original instrument cluster in bmw e46/e38/e39 chassis. For other cars with regular cluster, this isn't needed.


EasyEda project link for the PCB: https://easyeda.com/pazi88/ms42-43-compatible-speeduino-PnP-Core4

## Extra outputs on this board

### Rev 1.5
- Output pin 40 is wired to the electric thermostat. With help of programmable outputs the electric thermostat can be used to control engine temps.
  For exmaple make output activate when coolant is about 90°C and that should make the engine temps stay at 90°C instead of 100°C based on mechanical thermostat.
  Also you can add second condition to only do that above 80kPa to only lower temps at higher loads like stock ecu does.
- DISA valve is wired to the output pin 35, and it can be used by programmable outputs.
- Exhaust CAM vanos solenoid is wired to output pin 37, and it can be controlled by VVT control.
- Fan output pin is 36 and it's only wired to external connector fan output in the back of the ecu. The stock ecu connector fan output is controlled by bluepill in
  PWM fashion to be able to control stock fan. Note! that PWM fan feature is not yet implemented in bluepill WF.
### Rev 1.4 and older
- Output pin 35 is wired to the electric thermostat. With help of programmable outputs the electric thermostat can be used to control engine temps.
  For exmaple make output activate when coolant is about 90°C and that should make the engine temps stay at 90°C instead of 100°C based on mechanical thermostat.
  Also you can add second condition to only do that above 80kPa to only lower temps at higher loads like stock ecu does.
- DISA valve is wired to the output pin 37, and it can be used by programmable outputs.
- Exhaust CAM vanos solenoid is wired to output pin 36, and it can be controlled by VVT control.
- Default FAN output is wired to stock FAN output on the board and in addition to that, to the external connector. Note that stock FAN is PWM type and can't be
  controlled.
### Common in all versions
- The stock oil temp and radiator outlet temp sensors are also wired in by default, so those can be monitored using local aux in channels. Just solder JP3 and JP4
  to pull-up configuration(1-2) in CORE4 and those spare adc inputs work to monitor the oil and coolant outlet temps. (TBD: TS configuration instructions)
  Pin A13 is oil temp and pin A14 is coolant outlet temp.
- Exhaust CAM sensor is wired to input pin 21 (board default for VVT2).
- Output pin 32 goes to tank vent solenoid. So again this can be used by programmable outputs (activate when engine is running and above 90kPa). But this can
  be also used as easy way to add boost solenoid. The tank vent solenoid has 2-pin minitimer connector, so you can directly plug in VW n75 boost solenoid to
  replace that and use output pin 32 as boost output. And you got boost solenoid in engine bay, without extra wiring.
- Output pin 31 is wired to spare output at external connector
- Default tacho ouput is wired to stock ecu connector tacho output and also to external connector. If you have wired in the stock ms42/43 ecu to older car with
  traditional tacho, there is no need for wiring changes. e46/e39/e38 uses tacho trough CAN bus, so this output is not used in those. 
  NOTE! to use traditional tacho output, the JP6 needs to be soldered to pull-up configuration(1-2) in CORE4

## Speeduino code for Core4

The rev1.0/1.1 PCBs can use Speeduino FW without any changes.
Recommended way is to use Speedyloader to upload the Firmware to CORE4: https://wiki.speeduino.com/en/Installing_Firmware 

Rev1.2 onwards allows 6-cyl sequential injection and in order to run 6 cyl sequential, speeduino FW needs few customizations.
To upload use XLoader instead of Speedyloader: https://www.hobbytronics.co.uk/arduino-xloader Custom FW hex can be found from here: https://github.com/pazi88/Speeduino-M5x-PCBs/tree/master/6-cyl%20firmware%20files
Remember to select ATMEGA2560 as device. Also manual compiling and upload is option. To do that, check: https://github.com/pazi88/Speeduino-M5x-PCBs/tree/master/m52tu%2C%20m54%20PnP#compiling-speeduino-code-by-yourself

It's also recommended to change number of fuel outputs to 6 instead of 4 in speeduino.ini -file for Core4. But this isn't mandatory. It just gets rid of the possible
warnings in TS and allows to use HW test mode for injectors 5 and 6.

### Compiling speeduino code by yourself

If you want, you can also manually compile and upload the custom 6-cyl sequential FW. See manual compiling instructions at Wiki: https://wiki.speeduino.com/en/Installing_Firmware
Before compiling, change number of INJ_CHANNELS to 6 and number of IGN_CHANNELS to 3 in globals.h file:

![alt text](https://pazi88.kuvat.fi/kuvat/Projektikuvat/Random%20projektit/speeduino/Settings.png?img=smaller)

## Set solder jumpers for Core4

To get all the features working correctly for this board, set Core4 solder jumpers according to the pic:
![alt text](https://pazi88.kuvat.fi/kuvat/Projektikuvat/Random%20projektit/speeduino/jumpers.jpg?img=smaller)

## Compiling code for CAN-Bus interface

The CAN-bus interface uses STM32F103C8T6 bluepill board in conjunction of MCP2551 CAN transceiver to send CAN data to instrument cluster and read
data from speeduino using Serial3. In order to program the bluepill for CAN-interface, you will need FTDI breakout board and the code is meant to be 
used in Platform IO. Although it works in Arduino IDE too, if the compiler flags are set properly for it, but PIO is easier.

Steps to compile/upload code to the bluepill:

- Install Visual Studio Code: https://code.visualstudio.com/download
- Add Paltform IO to the VS Code: https://platformio.org/
- Download/clone this repository to your PC and open this folder in Platform IO (so you have the platformio.ini -file there)
- Use the Platform IO to edit the COM port numbers in platformio.ini to match COM port number for the FTDI breakout board.
- Click "Upload" and PIO should compile and upload the code to the bluepill (remember to set Boot0 jumper for the upload)
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/PIO.png?raw=true)

## USB drivers for Core4

Core4 uses Silicon Labs CP2102 USB-Serial chip for the PC connection. Windows and other OS's should automatically download and install correct drivers for it.
But if you are having problems with the USB drivers, the Core4 isn't detected etc., download and install drivers manually from here: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

## Reading data from CAN-bus to speeduino

The bluepill controlling the CAN-bus to car can also be used to read data back from CAN-bus to speeduino. To do this, enable Secondary Serial and Internal Canbus in TS.
Then go to "External Auxillary Input Channel Configuration" -menu. Following data is available, but it's not recommended to have all enabled. Only the ones that are really needed.
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/Aux_in.png?raw=true)
Note! This CAN-bus reading is fairly new addition in BluePill FW and might require updating it.

## Replacing original DBW throttle body

Because this board doesn't have DBW ability the original throttle body needs to be replaced with cable driven one in M54. M50/M52 throttle bodies
are easy replacement and adapters can be bought if needed. The DBW wiring needs to be wired to M50/M52 TPS according to this diagram:
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/M54-M50TPS.png?raw=true)

M52TU has cable drive semi-DBW system that can be used with this board without modifications. But if you want to replace the original throttle body
with M50/M52 one, follow this wiring diagram to adapt to M50/M52 TPS wiring:
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/M52TU-M50TPS.png?raw=true)

Wire color abbreviations:
- braun BR = brown
- blau BL = blue
- rot RT = red
- gelb GE = yellow
- grau GR = gray
- schwarz SW = black
- weiss (weiß ) WS = white
- grun GN = green
- orange OR = orange
- rosa RS = pink
- violett VI = violet

## Bosch LSU 4.9 Wideband sensor wiring

If internal Spartan 2 OEM or TinyWB wideband controller is used, it has 6-pin Molex Minifit Jr. Connecor with following pinout:
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/LSU49_connector.png?raw=true)
