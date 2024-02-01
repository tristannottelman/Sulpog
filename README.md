# Sulpog
Pokemon GO Plus emulator using ESP32 with display (fork).

![Alt text](Sulpogs.jpg?raw=true "Sulpogs")

## Features
This fork adds some features:

* Fully working GUI
* Statistics on screen: caught, spins and flees
* Displays timer
* Displays battery capacity
* 60 seconds to connect
* Stays on while charging
* Setting: turn on/off display
* Setting: change GUI color
* Setting: toggle between multiple PGP secrets (hold 'display on/off' and press power button)

## Known issues
* Battery will drain in sleep mode (unfortunately, you cannot turn the ESP32 off)
* Buttons on the case are very fragile

## Board
LilyGO TTGO T-Display with TFT Display

## Case
https://www.thingiverse.com/thing:4183337

## Battery
https://www.amazon.com/dp/B07CXNQ3ZR/ref=cm_sw_em_r_mt_dp_U_0XovEbVB7EPGM

## Get started
Run command to get a config file.
```
idf menuconfig
```

Select 'TFT_display_configuration' and then 'Select predefined display configuration' and choose the device you have. You can now build and flash the software.

Device specific configuration can be changed in `\components\tft\tftspi.h`.

## Commands
Build
```
idf build
```

Flash
```
idf -b 115200 -p <PORT> flash
```

Monitor
```
idf monitor -p <PORT>
```

## Links
Go Plus emulator\
https://github.com/yohanes/pgpemu

ESP32 TFT Display library\
https://github.com/loboris/ESP32_TFT_library
