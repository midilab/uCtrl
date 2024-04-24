#ifndef __U_CTRL_DEVICE_H__
#define __U_CTRL_DEVICE_H__

#include "../midi/midi.h"

namespace uctrl {

#define DEVICE_LABEL_SIZE 16
#define SYSEX_TEMPLATE_SIZE 16

// find a place for user to setup it
#define USE_DEVICE_LABELS
#define USE_EXT_RAM

// Sysex template system datatypes
typedef enum {
	SYSEX_TEMPLATE_STATIC,
	SYSEX_TEMPLATE_DYNAMIC,
	SYSEX_TEMPLATE_ROLAND_CHECKSUM,
	SYSEX_TEMPLATE_DATA_A,
	SYSEX_TEMPLATE_DATA_B,
	SYSEX_TEMPLATE_VALUE,
	SYSEX_TEMPLATE_TRACK_MIDI_CHANNEL
} MIDI_SYSEX_TEMPLATE_DATA_TYPE;		
			
typedef struct 
{
	uint8_t type;
	uint8_t value;
} MIDI_SYSEX_TEMPLATE_DATA;

typedef struct 
{
	uint8_t size;
	MIDI_SYSEX_TEMPLATE_DATA data[SYSEX_TEMPLATE_SIZE];
} MIDI_SYSEX_TEMPLATE;

//
// uControl Data Structure
//
typedef enum {
	DISABLE,
	MIDI_ANALOG,
	MIDI_TRIGGER,
	MIDI_BUTTON,
	//OSC_ANALOG,
	//OSC_TRIGGER,
	//OSC_BUTTON,
	//DMX_ANALOG,
	//DMX_TRIGGER,
	//DMX_BUTTON,
	APP_CTRL
} CONTROL_DATA_TYPE;

typedef struct
{
	uint8_t active;
	uint8_t port:3;    // 8 ports max for a previous allocated array of ports eg. MIDI1, MIDI2, USB1, USB2...
	uint8_t chn:5;     // MIDI Channel. 5 bits(16 channels + OMNI Channel)
	//uint8_t buffer_layout_id; // points to a BUFFER_LAYOUT structure data
	uint16_t buffer_size;
	uint16_t buffer_address; // buffer init point for a device data control	
#ifdef USE_DEVICE_LABELS
	#ifdef USE_EXT_RAM
	uint8_t name[DEVICE_LABEL_SIZE]; // go for ASCII only for now
	#else
	uint8_t * name;
	#endif
	uint16_t label_buffer_size;
	uint16_t label_buffer_address;
#endif	
} DEVICE_DATA;
// 6 bytes low resource

// when user request a map using deviceId and controlId
// deviceId can be defined by a init point address base and the control data can be access by this startAddress + controlId Offset
// eg. device 1 start data at buffer address X
// eg. device 2 start data at buffer address Y

// the main problems we can have?!
// runtime allocation only viable using a load all device data per session(devices are defined at session level.)
// eg. load first device: give first address as buffer_address. Allocate all data inside the buffer, take the last control data address base as reference for the second device buffer_address

typedef struct CONTROL_DATA CONTROL_DATA;
typedef struct CONTROL_DATA
{
	CONTROL_DATA_TYPE type;
	//uint8_t device_id:4;	// index reference for DEVICE_DATA memory area 	
	uint8_t config; 			// generic 8 bits flag map for config tunnings options
	uint8_t call;					// Event Call: MIDI command, OSC path, ...
	uint8_t data_a;				// Event Data A: MIDI data, OSC value 8bits
	uint8_t data_b;				// Event Data B: MIDI lsb, OSC value 8bits extension
	uint8_t data_c;				// Event Data C: MIDI msb, OSC value 8bits extension
	uint8_t data_d;				// Event Data D: Future implements... data_a + data_b + data_c + data_d = 32bits, maybe a 32 bits float point for some strange protocol as event data
#ifdef USE_DEVICE_LABELS
	#ifdef USE_EXT_RAM
	uint8_t name[DEVICE_LABEL_SIZE]; // go for ASCII only for now
	#else
	uint8_t * name;
	#endif
	uint16_t data_label; // buffer address of data label
	int16_t label_offset; // 
#endif // 20bytes
//	CONTROL_DATA * next; // we wont need this any more, resitered events have a phisical reference for knobs
} CONTROL_DATA;  
// 11bytes + 16bytes = 27bytes
// 9 bytes + 20 bytes = 29 bytes
// or we can took of the *next pointer for 7 bytes size bufer. but without the next we cant chain more than 1 event to the same physical control knob/fader

// 12 bytes / 32bits for parametrization data and 1 byte for call + 1 byte for interface parameters
// 12 bytes per event memory
// make the sysex buffer the same size
// create a memory area for sysex signatures indexed to be associated with event data that are of type sysex
// then just put the index inside data_d for example to load the template signature for event when it fire up it self
// Real Wolrd usage:
// each signature template is a template to control one hardware!
// some cases 2 or more signatures for one hardware... but in common way: one hardware!

// An option for display data for controls
// by type? 
// +numeric 		0-127 this is the default, 
// +percentage	0%-100%
// +fractionary 0-1/2-1/4-1/6...X?
// OK +range_user_defined_string		user setup a string for each range 0-126(OFF), 127(ON) or 0-12(1), 13-15(2), 16-127(Sync)

// customized actions by user
// Buttons
// name: filter type
// +increment by click(looped or not) Up and back, Up and down, Down and back, Down and up.
//    - click1 (send value X and shows "Hi-pass Filter")
//		- click2 (send value Y and shows "Low-pass Filter")

// Device parametrization (Roland MKS-30)
// +Port
// +Channel

// An memory area for event string data separattly to be created only when we register a display to use it.
// to be allocated dinamicly too, just like remote event buffer
// UTF-8 mapping
// 1 byte:       0 -     7F     (ASCII)
// 2 bytes:     80 -    7FF     (all European plus some Middle Eastern)
// 3 bytes:    800 -   FFFF     (multilingual plane incl. the top 1792 and private-use)
// 4 bytes:  10000 - 10FFFF

typedef struct DATA_LABEL DATA_LABEL;
typedef struct DATA_LABEL
{
	uint16_t range; // if control_value <= value: return this label. if not: check the next in chain... for this to work, you need to register label value in ascendent order of value
#ifdef USE_DEVICE_LABELS
	#ifdef USE_EXT_RAM
	uint8_t name[DEVICE_LABEL_SIZE]; // go for ASCII only for now
	#else
	uint8_t * name;
	#endif
#endif
	uint16_t next;
} DATA_LABEL;
// 4 bytes + 16 bytes = 20 bytes

typedef struct
{
#ifdef USE_EXT_RAM
	uint8_t device_name[DEVICE_LABEL_SIZE];
	uint8_t data_label[DEVICE_LABEL_SIZE];
	uint8_t ctrl_name[DEVICE_LABEL_SIZE];	
#else 
	uint8_t * device_name;
	uint8_t * data_label;
	uint8_t * ctrl_name;
#endif	
	uint8_t port;
	uint8_t device_id;
} CONTROL_DATA_INFO;

typedef struct
{
	uint16_t event_address:10; // change for a 10bit 1024 max. use 1023 instead of -1 value
	uint16_t device_id:5;	// 5 bits... 32 max
	uint16_t mode:1; 		// global or selected device
#ifndef LOW_RESOURCE_MCU	
	int16_t increment_base:11;	 // 11bits for a complement of 10bits signed signal
	int16_t value:11;	// change to uint16_t 1024 + no value registred
	uint16_t min:10; // ok lets change to 10bits because of adc resolution we cant go further
	uint16_t max:10; // the same here		
	// 12 bits left of uint16_t here
	uint8_t behaviour; // its a bitmask... no changes	
#endif	
} DIN_PORT_DATA;
// 12 bytes o_O
// 2 bytes

typedef struct
{
	uint16_t event_address:10;
	uint16_t device_id:5;	
	uint16_t mode:1; 		// global or selected device
	uint16_t min:10;
	uint16_t max:10;	
	//int16_t value;	12 bits left
} ADC_PORT_DATA;
// 3 bytes

typedef struct
{
	ADC_PORT_DATA * adc_port;
	DIN_PORT_DATA * din_port;
} REMOTE_DATA;

}

#endif
