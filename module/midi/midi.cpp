#include "../../../../modules.h"

#ifdef USE_MIDI

#include "midi.hpp"

namespace uctrl { namespace module {

//
// multicore archs
//
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	// mutex to protect the shared resource
	SemaphoreHandle_t _mutex;
	// mutex control for task
	#define ATOMIC(X) xSemaphoreTake(_mutex, portMAX_DELAY); X; xSemaphoreGive(_mutex);
	//portMUX_TYPE _uctrlMidiTimerMux = portMUX_INITIALIZER_UNLOCKED;
	//#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uctrlMidiTimerMux); X; portEXIT_CRITICAL_ISR(&_uctrlMidiTimerMux);
//
// singlecore archs
//
#else
	#define ATOMIC(X) noInterrupts(); X; interrupts();
#endif

Midi::Midi()
{
	_midiInputCallback = nullptr;
	_midiOutputCallback = nullptr;
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	_mutex = xSemaphoreCreateMutex();
#endif
}

Midi::~Midi()
{
	
}

template<typename T>
void Midi::initMidiInterface(T * device) {

	device->begin();

	// Setup MIDI Callbacks to handle incomming messages
	device->setHandleNoteOn(handleNoteOn);
	device->setHandleNoteOff(handleNoteOff);
	device->setHandleControlChange(handleCC);

	//device->setHandleAfterTouchPoly(handleAfterTouchPoly);
	//device->setHandleAfterTouchChannel(handleAfterTouchChannel);
	device->setHandleSystemExclusive(handleSystemExclusive);

	device->setHandleClock(handleClock);
	device->setHandleStart(handleStart);
	device->setHandleStop(handleStop);
}

void Midi::plug(midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> * device)
{
	_midi_port_if[_port_size] = (void *) device;

	initMidiInterface<midi::MidiInterface<midi::SerialMIDI<HardwareSerial>>>((midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> *)_midi_port_if[_port_size]);
	device->setHandlePitchBend(handlePitchBend);
	device->turnThruOff();	

	_port_size++;
}

#if defined(TEENSYDUINO) && !defined(__AVR_ATmega32U4__)
void Midi::plug(usb_midi_class * device)
{
	if ( _usb_port == 255 ) {
		_usb_port = _port_size;
		_midi_port_if[_usb_port] = (void *) device;

		initMidiInterface<usb_midi_class>((usb_midi_class *)_midi_port_if[_usb_port]);
		device->setHandlePitchChange(handlePitchBend);

		_port_size++;
	}
}
#elif defined(__AVR_ATmega32U4__)
void Midi::plug(midi::MidiInterface<usbMidi::usbMidiTransport> * device)
{
	if ( _usb_port == 255 ) {
		_usb_port = _port_size;
		_midi_port_if[_usb_port] = (void *) device;

		initMidiInterface<midi::MidiInterface<usbMidi::usbMidiTransport>>((midi::MidiInterface<usbMidi::usbMidiTransport> *)_midi_port_if[_usb_port]);
		device->setHandlePitchBend(handlePitchBend);
		device->turnThruOff();	

		_port_size++;
	}
}
#elif defined(CONFIG_TINYUSB_ENABLED) && (defined(ARDUINO_ARCH_ESP32) || defined(ESP32))
void Midi::plug(midi::MidiInterface<midi::SerialMIDI<ESPNATIVEUSBMIDI>> * device)
{
	if ( _usb_port == 255 ) {
		_usb_port = _port_size;
		_midi_port_if[_port_size] = (void *) device;

		initMidiInterface<midi::MidiInterface<midi::SerialMIDI<ESPNATIVEUSBMIDI>>>((midi::MidiInterface<midi::SerialMIDI<ESPNATIVEUSBMIDI>> *)_midi_port_if[_port_size]);
		device->setHandlePitchBend(handlePitchBend);
		device->turnThruOff();	

		_port_size++;
	}
}
#endif

/* 
#if defined(CONFIG_BT_ENABLED) && (defined(ARDUINO_ARCH_ESP32) || defined(ESP32))
void Midi::plug(midi::MidiInterface<bleMidi::BLEMIDI_Transport<bleMidi::BLEMIDI_ESP32>, bleMidi::MySettings>  * device)
{
	_midi_port_if[_port_size] = (void *) device;

	//initMidiInterface<midi::MidiInterface<bleMidi::BLEMIDI_Transport<bleMidi::BLEMIDI_ESP32>, bleMidi::MySettings>((midi::MidiInterface<bleMidi::BLEMIDI_Transport<bleMidi::BLEMIDI_ESP32>, bleMidi::MySettings> *)_midi_port_if[_port_size]);
	device->begin();

	// Setup MIDI Callbacks to handle incomming messages
	device->setHandleNoteOn(handleNoteOn);
	device->setHandleNoteOff(handleNoteOff);
	device->setHandleControlChange(handleCC);

	//device->setHandleAfterTouchPoly(handleAfterTouchPoly);
	//device->setHandleAfterTouchChannel(handleAfterTouchChannel);
	device->setHandleSystemExclusive(handleSystemExclusive);

	device->setHandleClock(handleClock);
	device->setHandleStart(handleStart);
	device->setHandleStop(handleStop);
	
	device->setHandlePitchBend(handlePitchBend);
	device->turnThruOff();

	// how to handle this for API access?
  	//BLEMIDI.setHandleConnected(OnConnected); // void OnConnected() {}
  	//BLEMIDI.setHandleDisconnected(OnDisconnected); // void OnDisconnected() {}

	_port_size++;
}
#endif
 */

uint8_t Midi::sizeOf()
{
	return _port_size;
}

void Midi::sendMessage(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t port, uint8_t interrupted, uint8_t config) 
{		
	if ( _midiOutputCallback != nullptr ) {
		_midiOutputCallback(msg, port, interrupted, config);
	} else {
		// Send data to midi_module driver interface
		if ( port == 0 ) {
			writeAllPorts(msg, interrupted);
		} else {
			write(msg, port, interrupted);
		}	
	}
}

template<typename T>
bool Midi::readMidiInterface(T * device) {
	return device->read();
}

// do only read if the port are not in realtime mode
bool Midi::read(uint8_t port)
{
	if ( port > _port_size || port == 0 ) {
		return false;
	}
	
	--port;
	
	_port = port;

	if ( _usb_port == port ) {
#if defined(TEENSYDUINO) && !defined(__AVR_ATmega32U4__)
		return readMidiInterface<usb_midi_class>((usb_midi_class *)_midi_port_if[port]);
#elif defined(__AVR_ATmega32U4__)
		return readMidiInterface<midi::MidiInterface<usbMidi::usbMidiTransport>>((midi::MidiInterface<usbMidi::usbMidiTransport> *)_midi_port_if[port]);
#elif defined(CONFIG_TINYUSB_ENABLED) && (defined(ARDUINO_ARCH_ESP32) || defined(ESP32))
		return readMidiInterface<midi::MidiInterface<midi::SerialMIDI<ESPNATIVEUSBMIDI>>>((midi::MidiInterface<midi::SerialMIDI<ESPNATIVEUSBMIDI>> *)_midi_port_if[port]);
#endif
	}

	// handle _ble_port too
	// if ( _ble_port == port ) { ... }

	return readMidiInterface<midi::MidiInterface<midi::SerialMIDI<HardwareSerial>>>((midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> *)_midi_port_if[port]);
}

void Midi::readAllPorts(uint8_t interrupted)
{
	static uint8_t counter;
	for (counter = 1; counter <= _port_size; counter++) {
		while (read(counter)) {
		}
	}
}

void Midi::writeAllPorts(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t interrupted)
{
	static uint8_t counter;
	for (counter = 1; counter <= _port_size; counter++) {
		write(msg, counter, interrupted);
	}
}

template<typename T>
void Midi::writeMidiInterface(T * device, uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t interrupted) {

    // packages with interrupted 1 are marked to be handled with interruptions disable to avoid MIDI override data with MIDI messages sent via some timer interruption

    // MIDI Handle
    switch (msg->type) {

		//Realtime
		case uctrl::protocol::midi::Clock:
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendRealTime(midi::Clock))
			} else {
				device->sendRealTime(midi::Clock);
			}
			break;   

		case uctrl::protocol::midi::Start:
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendRealTime(midi::Start))
			} else {
				device->sendRealTime(midi::Start);
			}
			break;  

		case uctrl::protocol::midi::Stop:
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendRealTime(midi::Stop))
			} else {
				device->sendRealTime(midi::Stop);
			}
			break;   

		// Non-realtime 
		case uctrl::protocol::midi::NoteOn:
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendNoteOn(msg->data1, msg->data2, msg->channel+1))
			} else {
				device->sendNoteOn(msg->data1, msg->data2, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::NoteOff:
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendNoteOff(msg->data1, 0, msg->channel+1))
			} else {
				device->sendNoteOff(msg->data1, 0, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::ControlChange:   
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendControlChange(msg->data1, msg->data2, msg->channel+1))
			} else {
				device->sendControlChange(msg->data1, msg->data2, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::ProgramChange:
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendProgramChange(msg->data1, msg->channel+1))
			} else {
				device->sendProgramChange(msg->data1, msg->channel+1);
			}
			break;
						
		case uctrl::protocol::midi::Nrpn:  
			if ( interrupted == 0 ) { 
				ATOMIC(
					// param select
					device->sendControlChange(99, 0x7f & (msg->data1 >> 7), msg->channel+1);
					device->sendControlChange(98, 0x7f & msg->data1, msg->channel+1);
					// send value
					device->sendControlChange(6, 0x7f & (msg->data2 >> 7), msg->channel+1);
					device->sendControlChange(38, 0x7f & msg->data2, msg->channel+1);
				)
			} else {
				// param select
				device->sendControlChange(99, 0x7f & (msg->data1 >> 7), msg->channel+1);
				device->sendControlChange(98, 0x7f & msg->data1, msg->channel+1);
				// send value
				device->sendControlChange(6, 0x7f & (msg->data2 >> 7), msg->channel+1);
				device->sendControlChange(38, 0x7f & msg->data2, msg->channel+1);
			}
			break;
		
		case uctrl::protocol::midi::PitchBend:
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendPitchBend(msg->data1, msg->channel+1))
			} else {
				//device->sendPitchBend((int16_t)(((uint8_t)msg->data1 << 8 ) | ((uint8_t)msg->data2 & 0xff)), msg->channel+1);
				device->sendPitchBend(msg->data1, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::Sysex:
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendSysEx(msg->data1, msg->sysex))
			} else {
				device->sendSysEx(msg->data1, msg->sysex);
			}
			break;


		case uctrl::protocol::midi::AfterTouchPoly:
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendPolyPressure(msg->data1, msg->data2, msg->channel+1))
			} else {
				device->sendPolyPressure(msg->data1, msg->data2, msg->channel+1);
			}
			break;
			
		case uctrl::protocol::midi::AfterTouchChannel:
			if ( interrupted == 0 ) { 
				ATOMIC(device->sendAfterTouch(msg->data1, msg->channel+1))
			} else {
				device->sendAfterTouch(msg->data1, msg->channel+1);
			}
			break;

		default:
			break;
    
    }
	
}

void Midi::write(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t port, uint8_t interrupted)
{
    // no outside range interface array access allowed here
    if ( port > _port_size || port == 0 ) {
        return;
	}
    
    --port;    

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	// since esp32 is a multicore arch we need to always make atomic! so force interrupted to 0
	interrupted = 0;
#endif

	if ( _usb_port == port ) {
#if defined(TEENSYDUINO) && !defined(__AVR_ATmega32U4__)
		writeMidiInterface<usb_midi_class>((usb_midi_class *)_midi_port_if[port], msg, interrupted);
#elif defined(__AVR_ATmega32U4__)
		writeMidiInterface<midi::MidiInterface<usbMidi::usbMidiTransport>>((midi::MidiInterface<usbMidi::usbMidiTransport> *)_midi_port_if[port], msg, interrupted);
#elif defined(CONFIG_TINYUSB_ENABLED) && (defined(ARDUINO_ARCH_ESP32) || defined(ESP32))
		writeMidiInterface<midi::MidiInterface<midi::SerialMIDI<ESPNATIVEUSBMIDI>>>((midi::MidiInterface<midi::SerialMIDI<ESPNATIVEUSBMIDI>> *)_midi_port_if[port], msg, interrupted);
#endif
		return;
	}

	writeMidiInterface<midi::MidiInterface<midi::SerialMIDI<HardwareSerial>>>((midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> *)_midi_port_if[port], msg, interrupted);
}

// MIDI Arduino library callbacks
void Midi::handleNoteOn(byte channel, byte pitch, byte velocity) 
{	
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.channel = channel;
		midi_module._message.data1 = pitch;
		midi_module._message.data2 = velocity;
		midi_module._message.type = uctrl::protocol::midi::NoteOn;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}
}

void Midi::handleNoteOff(byte channel, byte pitch, byte velocity) 
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.channel = channel;
		midi_module._message.data1 = pitch;
		midi_module._message.data2 = velocity;
		midi_module._message.type = uctrl::protocol::midi::NoteOff;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}	
}

void Midi::handleCC(byte channel, byte number, byte value)
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.channel = channel;
		midi_module._message.data1 = number;
		midi_module._message.data2 = value;
		midi_module._message.type = uctrl::protocol::midi::ControlChange;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}	
}

void Midi::handleAfterTouchPoly(byte channel, byte note, byte pressure)
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.channel = channel;
		midi_module._message.data1 = note;
		midi_module._message.data2 = pressure;
		midi_module._message.type = uctrl::protocol::midi::AfterTouchPoly;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}	
}

void Midi::handleAfterTouchChannel(byte channel, byte pressure)
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.channel = channel;
		//midi_module._message.data1 = note;
		midi_module._message.data2 = pressure;
		midi_module._message.type = uctrl::protocol::midi::AfterTouchChannel;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}		
}

void Midi::handlePitchBend(byte channel, int bend)
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.channel = channel;
		//midi_module._message.data1 = (uint8_t) (bend >> 8); // msb
		//midi_module._message.data2 = (uint8_t) bend & 0xff; // lsb
		midi_module._message.data1 = (int16_t) bend;	
		midi_module._message.type = uctrl::protocol::midi::PitchBend;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}		
}

void Midi::handleSystemExclusive(byte* array, unsigned size)
{
	
}

void Midi::handleClock()
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.type = uctrl::protocol::midi::Clock;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}	
}

void Midi::handleStart()
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.type = uctrl::protocol::midi::Start;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}	
}

void Midi::handleStop()
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.type = uctrl::protocol::midi::Stop;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}	
}

} }

uctrl::module::Midi midi_module;
#endif
