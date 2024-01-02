#ifndef __U_CTRL_MIDI_HPP__
#define __U_CTRL_MIDI_HPP__

#include <Arduino.h>

//
// multicore archs
//
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	// mutex to protect the shared resource
	static SemaphoreHandle_t _midi_mutex;
	// mutex control for task
	#define ATOMIC(X) xSemaphoreTake(_midi_mutex, portMAX_DELAY); X; xSemaphoreGive(_midi_mutex);
//
// singlecore archs
//
#else
	#define ATOMIC(X) noInterrupts(); X; interrupts();
#endif

// MIDI Support
#if defined(CONFIG_TINYUSB_ENABLED) && (defined(ARDUINO_ARCH_ESP32) || defined(ESP32))
#include <USB.h>
#include "ESPNATIVEUSBMIDI/ESPNATIVEUSBMIDI.h"
#endif

// want to use Bluetooth midi stack? keep in mind that consumes almost all of your code memory
#define USE_BT_MIDI_ESP32

#if defined(USE_BT_MIDI_ESP32) && defined(CONFIG_BT_ENABLED) && (defined(ARDUINO_ARCH_ESP32) || defined(ESP32))
#include "BLE-MIDI/BLEMIDI_Transport.h"
//#include "BLE-MIDI/hardware/BLEMIDI_ESP32_NimBLE.h"
#include "BLE-MIDI/hardware/BLEMIDI_ESP32.h"
//#include "BLE-MIDI/hardware/BLEMIDI_nRF52.h"
//#include "BLE-MIDI/hardware/BLEMIDI_ArduinoBLE.h"
#endif

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

    	template <typename T, typename U, typename V>
    	void plug(midi::MidiInterface<T, U, V> *midiInterface) {
			if (_port_size >= MAX_MIDI_DEVICE) {
				return;
			}

            midiArray[_port_size] = reinterpret_cast<void *>(midiInterface);
            readFunctions[_port_size] = &readImpl<T, U, V>;
            sendFunctions[_port_size] = &sendImpl<T, U, V>;

			// init interface
			midiInterface->begin();

			// Setup MIDI Callbacks to handle incomming messages
			midiInterface->setHandleNoteOn(handleNoteOn);
			midiInterface->setHandleNoteOff(handleNoteOff);
			midiInterface->setHandleControlChange(handleCC);
			//midiInterface->setHandleAfterTouchPoly(handleAfterTouchPoly);
			//midiInterface->setHandleAfterTouchChannel(handleAfterTouchChannel);
			midiInterface->setHandleSystemExclusive(handleSystemExclusive);
			midiInterface->setHandleClock(handleClock);
			midiInterface->setHandleStart(handleStart);
			midiInterface->setHandleStop(handleStop);

			//
			midiInterface->setHandlePitchBend(handlePitchBend);
			midiInterface->turnThruOff();	

			++_port_size;
		}

		using ReadFunction = void (*)(void *, uint8_t);
		//using SendFunction = void (*)(void *, const std::string &);
		using SendFunction = void (*)(void *, const midi::MidiType &, const midi::DataByte &, const midi::DataByte &, const midi::Channel &, uint8_t);
		void *midiArray[MAX_MIDI_DEVICE];
		ReadFunction readFunctions[MAX_MIDI_DEVICE];
		SendFunction sendFunctions[MAX_MIDI_DEVICE];
		template <typename T, typename U, typename V>
		static void readImpl(void *item, uint8_t interrupted) {
			midi::MidiInterface<T, U, V> *midi = reinterpret_cast<midi::MidiInterface<T, U, V> *>(item);
			//if (interrupted == 0) {
			//	ATOMIC(midi->read());
			//} else {
				midi->read();
			//}
		}
		template <typename T, typename U, typename V>
		static void sendImpl(void *item, const midi::MidiType &inType, const midi::DataByte &inData1,
							const midi::DataByte &inData2, const midi::Channel &inChannel, uint8_t interrupted) {
			midi::MidiInterface<T, U, V> *midi = reinterpret_cast<midi::MidiInterface<T, U, V> *>(item);
			//if (interrupted == 0) {
			//	ATOMIC(midi->send(inType, inData1, inData2, inChannel));
			//} else {
				midi->send(inType, inData1, inData2, inChannel);
			//}
		}
		/* template <typename T, typename U, typename V>
		static void readImpl(void *item, uint8_t interrupted);

		template <typename T, typename U, typename V>
		static void sendImpl(void *item, const midi::MidiType &inType, const midi::DataByte &inData1,
							 const midi::DataByte &inData2, const midi::Channel &inChannel, uint8_t interrupted);
		 */

		bool read(uint8_t port);
		void readAllPorts(uint8_t interrupted = 0);
		void write(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t port, uint8_t interrupted = 0);
		void writeAllPorts(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t interrupted = 0);
		uint8_t sizeOf();	

		void writeMidiInterface(uint8_t port, uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t interrupted = 0);

        uint8_t _port_size = 0;
        uint8_t _port = 0;
		bool _ble_connected = false;

        uctrl::protocol::midi::MIDI_MESSAGE _message;

		// keep only the address of devices, type conversion is done in compile time 
		// using templates of initMidiInterface<>(), writeMidiInterface<>(), readMidiInterface<>()
		//void * _midi_port_if[MAX_MIDI_DEVICE] = {nullptr};

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
