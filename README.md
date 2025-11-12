## RFM69HCW Radiolib for ESP32-S3

This code modifies the example [Transmit](https://registry.platformio.org/libraries/jgromes/RadioLib/examples/RF69/RF69_Transmit_Interrupt/RF69_Transmit_Interrupt.ino) and [Receive]((https://registry.platformio.org/libraries/jgromes/RadioLib/examples/RF69/RF69_Receive_Interrupt/RF69_Receive_Interrupt.ino)) 
to work on the ESP32-S3 board.

This was tested with a ESP32-S3-DevKitC-1

Uses the Arduino framework under PlatformIO


#### To Trasmit
Change lines 23-24 in main.cpp:
```c
#define TRANSMIT_MODE
//#define RECEIVE_MODE
```

#### To Receive
Change lines 23-24 in main.cpp:
```c
//#define TRANSMIT_MODE
#define RECEIVE_MODE
```
