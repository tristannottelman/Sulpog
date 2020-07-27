# Sulpog
Go Plus implementation with display

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
