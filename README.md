# uCtrl

uCtrl is the base library of [uMODULAR](https://github.com/midilab/uMODULAR) hardware project. It provides driver layer for all uMODULAR modules and user interface layer to easly create, extend and share more advanced Arduino applications.

This library also enables realtime-like functionality inside Arduino ecosystem by using hardware timer interruption to create time predictable task management.

uCtrl is a choice for robust, portable and fast-making musical instruments, sequencers, audio/video controllers and other related machines for Arduino platform.

## Supported microcontrollers

The following microcontrollers and boards are supported and were tested:

**AVRs:** ATmega168, ATmega328, ATmega16u4, ATmega32u4 and ATmega2560.  
**ARMs:** ESP32, SAMD21 and All Teensy ARM microcontrollers.  

**Boards:** All AVR Arduino boards, All Teensy AVRs and ARMs, ESP32 based boards and Seedstudio XIAO M0.

## Modules avaliable

[PAGE](#page): Modular User Interface programming using pages and components.  
[AIN](#ain): Wire up to 64 potentiometers.  
[DIN](#din): Wire up to 64 push buttons or rotary encoders.  
[DOUT](#dout): Wire up to 64 leds.  
[TOUCH](#touch): Wire up to 32 capcitive touch buttons(teensy arms only for now).  
[MIDI](#midi): Agregate and control MIDI interfaces.  
[OLED](#oled): Connect a OLED screen.  
[STORAGE](#storage): Make use of epprom and/or connect a sdcard.  
[RAM](#ram): Make use of external SRAM.  

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

Create a src/ directory inside your sketch, then clone or [download](https://github.com/midilab/uCtrl/archive/refs/heads/main.zip) this repository.

unzip main.zip and move the unziped folder named uCtrl-main/ to YourSketch/src/uCtrl/

### Setting your sketch

Create a new file on IDE tab for your sketch named **modules.h**

Configure modules.h accordly to the needed modules for your application.(see Modules for MACRO setup)

### modules.h

You should have a guard macro for this file:

```c
#ifndef __U_CTRL_MODULES_H__
#define __U_CTRL_MODULES_H__

// your uCtrl modules MACRO setup

#endif
```

### Initial Project structure 

Your initial project structure should look like:  
/YourSketch  
/YourSketch/YourSketch.ino  
/YourSketch/modules.h  
/YourSketch/src/uCtrl/  

# Modules

A introduction to modules, base schematics and modules.h MACRO setup.

## AIN

Wire up to 64 potentiometers.  

Good to create midi controllers or user interface with potentiometers.

This module can handle single ADC ports on your microcontroller or multiplexed ADC port via 4051 CI.

modules.h
```c
#ifndef __U_CTRL_MODULES_H__
#define __U_CTRL_MODULES_H__

#define USE_AIN

#endif
```

YourSketch.ino
```c++
#include <Arduino.h>
#include "src/uCtrl/uCtrl.h"

// get change values from connected potentiometers
void ainInput(uint8_t port, uint16_t value, uint8_t interrupted)
{
    switch (port) {
        case 1:
            // do something with port 1 value changing
            break;
        case 2:
            // do something with port 2 value changing
            break;
    }
}

void setAinMultiplexed()
{
    // MUX_CTRL_PIN_* are the control pins of 4051 CI:
    // datasheet link
    // initAin(uint8_t MUX_CTRL_PIN_A, uint8_t MUX_CTRL_PIN_B, uint8_t MUX_CTRL_PIN_C)
    uCtrl.initAin(2, 3, 4);
    // plug(uint8_t MUX_ANALOG_PORT_PIN_X)
    uCtrl.ain->plug(A0);
    uCtrl.ain->plug(A1);
    uCtrl.ain->plug(A2);
}

void setAinSingle()
{
    uCtrl.initAin();
    // plug(uint8_t MUX_ANALOG_PORT_PIN_X)
    uCtrl.ain->plug(A1);
    uCtrl.ain->plug(A0);
    uCtrl.ain->plug(A2);
}

void setup() 
{
    setAinSingle();
    //setAinMultiplexed();
    uCtrl.ain->setCallback(ainInput);
    // most arduinos default is 1024 based on your internal ADC resolution 
    uCtrl.ain->setMaxAdcValue(128);
}

void loop()
{
  uCtrl.run();
}
```

Make use of 4051 and/or 4067(to be supported) for multiplexed analog inputs
*link or image to the ain options schematic

## DIN

Wire up to 64 push buttons or rotary encoders.

Good to create midi controllers or user interface interactions with potentiometers.

This module can handle single ADC ports on your microcontroller or multiplexed ADC port via 4051 CI.

modules.h
```c
#ifndef __U_CTRL_MODULES_H__
#define __U_CTRL_MODULES_H__

#define USE_DIN

//
// for direct usage of microcontroller ADC port pin
// USE_DIN_MAX_PORTS is default to 16 if you dont set it
// if you need less than 16 please set it accordly to save memory
//
#define USE_DIN_PORT_PIN
//#define USE_DIN_MAX_PORTS   8

// two driver options for multiplexed button input, via SPI and via BITBANG.
// uncomment only one driver option.

//
// using SPI hardware wich is the recommended way in case you have a Free SPI device.
//
//#define USE_DIN_SPI_DRIVER
//#define DIN_LATCH_PIN   D4

//
// using bitbang in case you dont have a free SPI. 
// this is slower and uses the CPU to process the data transfer.
// this options requires you to define the latch, data and clock pins of 4051.
//
//#define USE_DIN_BITBANG_DRIVER
//#define DIN_LATCH_PIN   D4
//#define DIN_DATA_PIN    D5
//#define DIN_CLOCK_PIN   D6

#endif
```

YourSketch.ino
```c++
#include <Arduino.h>
#include "src/uCtrl/uCtrl.h"

// get change values from connected push buttons or encoders
void dinInput(uint8_t port, uint16_t value, uint8_t interrupted)
{
    switch (port) {
        case 1:
            // do something with port 1 value changing
            break;
        case 2:
            // do something with port 2 value changing
            break;
    }
}

void setDinMultiplexedSpi()
{
    // initDin(spi device)
    uCtrl.initDin(&SPI);
    // plugSR(uint8_t number of 165's to plug)
    uCtrl.din->plugSR(1);
}

void setDinMultiplexedBitbang()
{
    uCtrl.initDin();
    // plugSR(uint8_t number of 165's to plug)
    uCtrl.din->plugSR(1);
}

void setDinSingle()
{
    uCtrl.initDin();
    // plug(uint8_t MUX_ANALOG_PORT_PIN_X)
    uCtrl.din->plug(D2);
    uCtrl.din->plug(D3);
    uCtrl.din->plug(D4);
}

void setup() 
{
    setDinSingle();
    //setDinMultiplexedSpi();
    //setDinMultiplexedBitbang();
    uCtrl.din->setCallback(dinInput);
    // most arduinos default is 1024 based on your internal ADC resolution 
    uCtrl.din->setMaxAdcValue(128);
}

void loop()
{
  uCtrl.run();
}
```

Make use of 165 shift register to expand buttons and/or encoders.
*link or image to the din options schematic

## DOUT

Wire up to 64 leds.

Make use of 595 shiftregister to expand digital output like leds
*link or image to the dout options schematic

## Touch

Wire up to 32 touch buttons

Make use of 4067 to multiplex capacitive buttons
*link or image to the touch options schematic

## MIDI

Agregate and control MIDI interfaces.

Agregator for [author name] midi library for realtime usage
*link or image to the midi options schematic

## Oled

Connect a OLED screen.

Helper class that makes use of [author name] oled library
*link or image to the oled options schematic

## Storage

Make use of epprom and/or sdcard.

Storage abstration for uCtrl using Epprom and Sdcard(library from..)
*link or image to the sdcard options schematic

## Ram

(23LC1024 only for now)

## Page

Environment programming to interface your app using the ecosystem of uCtrl, can be used with components for interface
