#ifndef __U_CTRL_MIDI_HPP__
#define __U_CTRL_MIDI_HPP__

#include <Arduino.h>

// MIDI Support
#include "MIDI/MIDI.h"
#if defined(__AVR_ATmega32U4__)
#include "USB-MIDI/USB-MIDI.h"
#endif

#include "midi.h"
	
namespace uctrl { namespace module { 

#define MAX_MIDI_DEVICE 6

class Midi
{
    public:
    
        Midi();
        ~Midi();

		void init();	
		void plug(midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> * device);
#if defined(TEENSYDUINO) && !defined(__AVR_ATmega32U4__)
		void plug(usb_midi_class * device);
#elif defined(__AVR_ATmega32U4__)
		void plug(midi::MidiInterface<usbMidi::usbMidiTransport> * device);
#endif
		bool read(uint8_t port);
		void readAllPorts(uint8_t interrupted = 0);
		void write(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t port, uint8_t interrupted = 0);
		void writeAllPorts(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t interrupted = 0);
		uint8_t sizeOf();	

		template<typename T>
		void initMidiInterface(T * device);

		template<typename T>
		void writeMidiInterface(T * device, uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t interrupted);

		template<typename T>
		bool readMidiInterface(T * device);

        uint8_t _port_size = 0;
        uint8_t _port = 0;
		uint8_t _usb_port = 255;

        uctrl::protocol::midi::MIDI_MESSAGE _message;

		// keep only the address of devices, type conversion is done in compile time 
		// using templates of initMidiInterface<>(), writeMidiInterface<>(), readMidiInterface<>()
		void * _midi_port_if[MAX_MIDI_DEVICE] = {nullptr};

		void sendMessage(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t port, uint8_t interrupted = 0, uint8_t config = 0);

		// MIDI data handler
		void (*_midiInputCallback)(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t port, uint8_t interrupetd);
		void setMidiInputCallback(void (*callback)(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t port, uint8_t interrupetd)) {
			_midiInputCallback = callback;
		}
		void (*_midiOutputCallback)(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t port, uint8_t interrupetd, uint8_t config);
		void setMidiOutputCallback(void (*callback)(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t port, uint8_t interrupetd, uint8_t config)) {
			_midiOutputCallback = callback;
		}
	
		// because of MIDI library way of working we need static methods with static data.
		static void handleNoteOn(byte channel, byte pitch, byte velocity);
		static void handleNoteOff(byte channel, byte pitch, byte velocity);
		static void handleAfterTouchPoly(byte channel, byte note, byte pressure);
		static void handleAfterTouchChannel(byte channel, byte pressure);
		static void handlePitchBend(byte channel, int bend);
		static void handleSystemExclusive(byte* array, unsigned size);
		static void handleCC(byte channel, byte number, byte value);
		static void handleClock();
		static void handleStart();
		static void handleStop();		

};

} }

extern uctrl::module::Midi midi_module;

#endif
