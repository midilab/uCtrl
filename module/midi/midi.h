#ifndef __U_CTRL_MIDI_H__
#define __U_CTRL_MIDI_H__

//#include "MIDI/midi_Defs.h"

namespace uctrl { namespace protocol { namespace midi {

typedef enum {
	ControlChange,
	ProgramChange,			
	NoteOn,
	NoteOnHold,
	NoteOff,
	AfterTouchPoly,
	AfterTouchChannel,
	PitchBend,
	Nrpn,
	Sysex,
	Clock,
	Start,
	Stop
} MidiMessageType;	

/*
    InvalidType           = 0x00,    ///< For notifying errors
    NoteOff               = 0x80,    ///< Note Off
    NoteOn                = 0x90,    ///< Note On
    AfterTouchPoly        = 0xA0,    ///< Polyphonic AfterTouch
    ControlChange         = 0xB0,    ///< Control Change / Channel Mode
    ProgramChange         = 0xC0,    ///< Program Change
    AfterTouchChannel     = 0xD0,    ///< Channel (monophonic) AfterTouch
    PitchBend             = 0xE0,    ///< Pitch Bend
    SystemExclusive       = 0xF0,    ///< System Exclusive
    TimeCodeQuarterFrame  = 0xF1,    ///< System Common - MIDI Time Code Quarter Frame
    SongPosition          = 0xF2,    ///< System Common - Song Position Pointer
    SongSelect            = 0xF3,    ///< System Common - Song Select
    TuneRequest           = 0xF6,    ///< System Common - Tune Request
    Clock                 = 0xF8,    ///< System Real Time - Timing Clock
    Start                 = 0xFA,    ///< System Real Time - Start
    Continue              = 0xFB,    ///< System Real Time - Continue
    Stop                  = 0xFC,    ///< System Real Time - Stop
    ActiveSensing         = 0xFE,    ///< System Real Time - Active Sensing
    SystemReset           = 0xFF,    ///< System Real Time - System Reset
 */ 
typedef struct 
{
	MidiMessageType type;
    //midi::MidiType type;
	uint8_t channel;
	int16_t data1; // 16bits for NRPN support
	int16_t data2; // 16bits for NRPN support
	uint8_t sysex[16];
} MIDI_MESSAGE;	
		
} } }

#endif
