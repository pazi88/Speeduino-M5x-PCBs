# Speeduino Compatible PCBs for BMW M52TU and M54 engines
![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/rev12.jpg?raw=true)

This is folder for Speeduino Compatible PCBs that can be used with BMW M52TU and M54 engine in PnP fashion. This PCB will replace the
original board in ms42/ms43 ecu case. Just open the 4 bolts in the ecu case, remove original board and replace with this. Last step is
to carve some openings for the extra connectors and the ecu can be plugged into car.

NOTE! DBW is not supported, so in M54 engine (ms43), you need to change to cable driven throttle body in order to use this.
M52TU (ms42) has cable driven throttle as backup, so no changes needed in those to use this speeduino board. (TBD: add cable throttle instructions)

NOTE 2! OBD2 system is not supported in these boards, so if your country requires the car to pass OBD2 tests in MOT check, this board can't be used.

![alt text](https://github.com/pazi88/Speeduino-M5x-PCBs/blob/master/m52tu,%20m54%20PnP/Pics/withcase.jpg?raw=true)

Some of the features this PCB has:
- Compatible with Speeduino FW (rev1.0/1.1 are directly compatible. Rev1.2/1.3 require small changes.)
- 6-cyl sequential fuel in rev1.2/1.3
- Uses DIY-EFI Core4 ECU module, instead of Arduino Mega. For more info: https://diy-efi.co.uk/product/core4-module
- Provision to use Spartan2 OEM wideband controller. For more info: https://www.14point7.com/products/spartan-2-oem
- Provision to use DIY-EFI TinyWB wideband controller. (rev1.3)
- Includes CAN-bus interface to control original instrument cluster in bmw e46/e38/e39 chassis. For other cars with regular cluster, this isn't needed.

## Extra outputs on this board

- Output pin 35 is wired to the electric thermostat. With help of programmable outputs in 202008 FW the electric thermostat can be used to control engine temps.
  For exmaple make output activate when coolant is about 90°C and that should make the engine temps stay at 90°C instead of 100°C based on mechanical thermostat.
  Also you can add second condition to only do that above 80kPa to only lower temps at higher loads like stock ecu does.
- Output pin 32 goes to tank vent solenoid. So again this can be used by programmable outputs (activate when engine is running and above 90kPa). But this can
  be also used as easy way to add boost solenoid. The tank vent solenoid has 2-pin minitimer connector, so you can directly plug in VW n75 boost solenoid to
  replace that and use output pin 32 as boost output. And you got boost solenoid in engine bay, without extra wiring.
- The stock oil temp and radiator outlet temp sensors are also wired in by default, so those can be monitored using local aux in channels. Just solder JP3 and JP4
  to pull-up configuration(1-2) in CORE4 and those spare adc inputs work to monitor the oil and coolant outlet temps. (TBD: TS configuration instructions)
  Pin A13 is oil temp and pin A14 is coolant outlet temp.
- DISA valve is wired to the output pin 37, and it can be used by programmable outputs.
- Exhaust CAM vanos solenoid is wired to output pin 36, and it can be used by programmable outputs.
- Output pin 31 is wired to spare output at external connector
- Default FAN output is wired to stock FAN output on the board and in addition to that, to the external connector. Note that stock FAN is PWM type and can't be
  controlled by current FW versions.
- Default tacho ouput is wired to stock ecu connector tacho output and also to external connector. If you have wired in the stock ms42/43 ecu to older car with
  traditional tacho, there is no need for wiring changes. e46/e39/e38 uses tacho trough CAN bus, so this output is not used in those. 
  NOTE! to use traditional tacho output, the JP6 needs to be soldered to pull-up configuration(1-2) in CORE4

## Compiling speeduino code for Core4

The rev1.0/1.1 PCBs can use Speeduino FW without any changes. Just use 201909 speeduino FW release or newer and select DIY-EFI CORE4 v1.0 as board layout in TS.
Recommended way is to use Speedyloader to upload the Firmware to CORE4: https://wiki.speeduino.com/en/Installing_Firmware 

Rev1.2 onwards allows 6-cyl sequential injection and in order to run those 202005 or later speeduino FW is needed. Also the easy Speedyloader 
FW upload can't be used but it requires manual compiling with small changes in code. See manual compiling instructions in Wiki at earlier link. 
Before compiling, change number of INJ_CHANNELS to 6 and number of IGN_CHANNELS to 3 in globals.h file:

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
