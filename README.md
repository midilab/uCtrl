# uCtrl

uCtrl is a comprehensive codebase library for the [uMODULAR](https://github.com/midilab/uMODULAR) hardware project. It offers a driver layer for all uMODULAR modules and a user interface layer that simplifies the creation, extension, and sharing of advanced Arduino/PlatformIO applications.  
  
With uCtrl, you can achieve realtime-like functionality within the Arduino/PlatformIO ecosystem by leveraging hardware timer interruptions for precise task management. The library provides a resource-safe access through its API, ensuring efficient and reliable operation.  
  
Choose uCtrl for developing robust, portable, and high-performance musical instruments, sequencers, audio/video controllers, and other related machines on the Arduino/PlatformIO platform.  
  
## Supported microcontrollers

The following microcontrollers and boards are supported and were tested:

**AVRs:** ATmega168, ATmega328, ATmega16u4, ATmega32u4 and ATmega2560.  
**ARMs:** ESP32, SAMD21 and All Teensy ARM microcontrollers(1.57.2).  

**Boards:** All AVR Arduino boards, All Teensy AVRs and ARMs, ESP32 based boards and Seedstudio XIAO M0.

## Modules avaliable

[AIN](#ain): Wire up to 64 potentiometers.  
[DIN](#din): Wire up to 64 push buttons or rotary encoders.  
[DOUT](#dout): Wire up to 64 leds or any other digital output.  
[TOUCH](#touch): Wire up to 32 capcitive touch buttons.  
[MIDI](#midi): Agregate and control MIDI interfaces.  
[OLED](#oled): Connect a OLED screen.  
[STORAGE](#storage): Make use of epprom and/or connect a sdcard.  
[RAM](#ram): Make use of external SRAM.  
[PAGE](#page): Modular User Interface programming using pages and components.  

## How to use?

In case you are using git for your Arduino application we strongly suggest to setup uCtrl library for your project as a [submodule](#submodule).

If you dont need versionng control for you Arduino application the fast way to setup you uCtrl library for your project is via [download](#download).

### Submodule

This allows you to manage uCtrl version related to your project version.

```console
$ cd YourSketch/
$ mkdir src/
$ cd src/
$ git submodule add https://github.com/midilab/uCtrl.git
$ git add uCtrl/
$ git commit -m "uCtrl library added to project"
$ git push
```

after each new clone of your application repository you'll need to init subproject uCtrl only once:

```console
$ git clone https://github.com/mister_dev/YourSketch.git
$ cd YourSketch/
$ git submodule update --init --recursive
```

### Download

In case you want to download uCtrl instead of use it as a submodule.  

Create a src/ directory inside your sketch, then clone or [download](https://github.com/midilab/uCtrl/archive/refs/heads/main.zip) this repository.

unzip **main.zip** and move the unziped folder named **uCtrl-main/** to **YourSketch/src/uCtrl/**

### Initial Project structure 

Your initial project structure should look like:  
/YourSketch  
/YourSketch/YourSketch.ino  
/YourSketch/src/uCtrl/  

# Modules

A introduction to modules and base schematics setup.

All the schematics are available as [PDF or Kicad format here](https://github.com/midilab/uMODULAR/tree/master/lib)

## AIN

Wire up to 64 potentiometers.  

Use this module to create midi controllers or any other user interface with potentiometers.

This module can handle single ADC ports on your microcontroller or multiplexed ADC ports via 4051 or 4067 CI.

[AIN 8 Potentiometers Schematic](https://github.com/midilab/uMODULAR/blob/master/lib/pot8.pdf)  
[AIN 16 Potentiometers Schematic](https://github.com/midilab/uMODULAR/blob/master/lib/pot16.pdf)

YourSketch.ino
```c++
#include <Arduino.h>
#include "src/uCtrl/uCtrl.h"

enum {
  POT_1,
  POT_2,
  POT_3,
  POT_4,
  //...,
  //...,
};

// get change values from connected potentiometers
void ainInput(uint8_t port, uint16_t value)
{
  switch (port) {
    case POT_1:
      // do something with port 1 value(0 ~ 1023 || setMaxAdcValue)
      break;
    case POT_2:
      // do something with port 2 value(0 ~ 1023 || setMaxAdcValue)
      break;
    case POT_3:
      // do something with port 3 value(0 ~ 1023 || setMaxAdcValue)
      break;
    case POT_4:
      // do something with port 4 value(0 ~ 1023 || setMaxAdcValue)
      break;
    //...
    //...
  }
}

void setup() 
{
  // always call module init before setup it
  // If you're using 4051 or 4067 multiplexers initAin with
  // mux control pins to be used as parameters
  // for 4051
  // initAin(int mux_ctrl_a, int mux_ctrl_b, int mux_ctrl_c)
  uCtrl.initAin(6,7,8);
  // for 4067
  // initAin(int mux_ctrl_a, int mux_ctrl_b, int mux_ctrl_c, int mux_ctrl_d)
  //uCtrl.initAin(6,7,8,9);
  // for microcontroller analog ports only(no mux)
  //uCtrl.initAin();
  
  // just plug the hardware by analog ports ADC
  // plug() is for microcontroller direct ADC pin
  // this example adds two ADC ports making 2 potentiometers avaliable for use
  uCtrl.ain->plug(A0);
  uCtrl.ain->plug(A1);
  // plugMux() is for multiplexed ADC pin read via 4051/4067
  // this examples adds two 4051(see modules for 4067) making 16 potentiometers avaliable for use
  uCtrl.ain->plugMux(A2);
  uCtrl.ain->plugMux(A3);

  // total 18 potentiometers avaliable: 2 direct connected and 16 via 2x 4051
  
  // callback where you receive potentiometer changes
  uCtrl.ain->setCallback(ainInput);
  // you can use this RT callback in case too much processing is done
  // on your applicatiuon side and that took effect on changes speed
  // but use this only if needed! dealing with RT means dealing with shared resources
  //uCtrl.ain->setRTCallback(ainInput);
  
  // most arduinos max is 1024 and also the default if not set
  // you can lower or raiser this value for your needs.
  // midi controllers only need max 128(if you dont plan to make use of NRPN or sysex extensions)
  uCtrl.ain->setMaxAdcValue(128);
  // the more you read input samples the lower the noise, but more processing too.
  // use only if needed and dont go too far!
  uCtrl.ain->setAvgReads(8);
  
  // only init uCtrl after all modules setup
  uCtrl.init();
}

void loop()
{
  uCtrl.run();
}
```

## DIN

Wire up to 64 push buttons or rotary encoders.

This module can handle single Digital Input ports on your microcontroller or multiplexed Digital Input port via 165 CI.

[DIN 8 Push Buttons Schematic](https://github.com/midilab/uMODULAR/blob/master/lib/push8.pdf)  
[DIN 16 Push Buttons Schematic](https://github.com/midilab/uMODULAR/blob/master/lib/push16.pdf)

YourSketch.ino
```c++
#include <Arduino.h>
#include "src/uCtrl/uCtrl.h"

enum {
  BUTTON_1,
  BUTTON_2,
  ENCODER_DEC,
  ENCODER_INC,
  SR_BUTTON_1,
  SR_BUTTON_2,
  SR_BUTTON_3,
  SR_BUTTON_4,
  //...,
  //...,
};

// get change values from connected push buttons or encoders
void dinInput(uint8_t port, uint16_t value)
{
  switch (port) {
    case BUTTON_1:
      // do something with BUTTON_1 value(HIGH | LOW)
      break;
    case BUTTON_2:
      // do something with BUTTON_2 value(HIGH | LOW)
      break;
    case ENCODER_DEC:
      // do something with ENCODER_DEC value(HIGH)
      break;
    case ENCODER_INC:
      // do something with ENCODER_INC value(HIGH)
      break;
    case SR_BUTTON_1:
      // do something with SR_BUTTON_1 value(HIGH | LOW)
      break;
    case SR_BUTTON_2:
      // do something with SR_BUTTON_2 value(HIGH | LOW)
      break;
    case SR_BUTTON_3:
      // do something with SR_BUTTON_3 value(HIGH | LOW)
      break;
    case SR_BUTTON_4:
      // do something with SR_BUTTON_4 value(HIGH | LOW)
      break;
    //...
    //...
  }
}

void setup() 
{
  // if you're going to use SPI driver you need to pass the 
  // spi device you want to initDin and latch_pin to be used
  // for 165 shiftregisters
  // initDin(spi device, int latch_pin)
  uCtrl.initDin(&SPI, 6);
  // otherwise you can just initDin without parameters
  //uCtrl.initDin();
  
  uCtrl.din->plug(13);
  uCtrl.din->plug(12);
  uCtrl.din->plug(26);
  uCtrl.din->plug(28);
  // this plugs 4 microntroller Digital ports making 4 push buttons avaliable

  // this plugs 1x 165, making 8 push buttons avaliable
  // plugSR(uint8_t number of 165's to plug)
  uCtrl.din->plugSR(1);
  
  uCtrl.din->setCallback(dinInput);
  
  // encoders setup?
  // in pair and sequential pins always! 
  // pairs starting with even ids: 0 and 1, 2 and 3, 4 and 5
  // call it only after all plug() and plugSR() requests
  uCtrl.din->encoder(ENCODER_DEC, ENCODER_INC);
  
  // only init uCtrl after all modules setup
  uCtrl.init();
}

void loop()
{
  uCtrl.run();
}
```

## DOUT

Wire up to 64 leds or any other digital output.

This module can handle single Digital output ports on your microcontroller or multiplexed Digital output port via 595 CI.

[DOUT 8 Leds Schematic](https://github.com/midilab/uMODULAR/blob/master/lib/led8.pdf)  
[DOUT 16 Leds Schematic](https://github.com/midilab/uMODULAR/blob/master/lib/led16.pdf)

YourSketch.ino
```c++
#include <Arduino.h>
#include "src/uCtrl/uCtrl.h"

enum {
  ON_BOARD_LED_1,
  ON_BOARD_LED_2,
  SR_LED_1,
  SR_LED_2,
  SR_LED_3,
  SR_LED_4,
  //...,
  //...,
};

void setup() 
{
  // if you're going to use SPI driver you need to pass the 
  // spi device you want to initDout and latch_pin to be used
  // for 595 shiftregisters
  // initDout(spi device, int latch_pin)
  uCtrl.initDout(&SPI, 8);
  // otherwise you can just initDout without parameters
  //uCtrl.initDout();
  
  // this plugs 2 microntroller Digital ports making 2 digital outputs avaliable
  uCtrl.dout->plug(5);
  uCtrl.dout->plug(6);

  // this plugs 1x 595, making 8 digital outputs avaliable
  // plugSR(uint8_t number of 595's to plug)
  uCtrl.dout->plugSR(1);

  // only init uCtrl after all modules setup
  uCtrl.init();

  // now you can use the uCtrl interface to set states on the outputs
  // set all leds off
  uCtrl.dout->writeAll(LOW); 
  // set all leds on
  //uCtrl.dout->writeAll(HIGH); 
}

void loop()
{
  uCtrl.run();

  // set SR_LED_1 on
  uCtrl.dout->write(SR_LED_1, HIGH); 
}
```

## Touch

Wire up to 32 capacitive touch buttons using common ADC port.

Make use of 4067 to multiplex capacitive buttons.  
*link or image to the touch options schematic

## MIDI

Agregate and control MIDI interfaces.
Agregator for [Francois Best's Arduino MIDI library](https://github.com/FortySevenEffects/arduino_midi_library) that can handle realtime operations safely inside uCtrl.  

Make use of MIDI circuit.
*link or image to the midi options schematic

## Oled

Connect a OLED screen.

Display helper class for [U8g2 oled library](https://github.com/olikraus/U8g2_Arduino). 


Make use of i2c or spi devices.
*link or image to the oled options schematic  

## Storage

Make use of epprom and/or sdcard.

Storage abstraction for for Epprom and Sdcard usage. Sdcard library helper using [greiman's SdFat library](https://github.com/greiman/SdFat/)
*link or image to the sdcard options schematic

## Ram

(23LC1024 only for now)

## Page

Environment programming to interface your app using the ecosystem of uCtrl, can be used with components for interface
