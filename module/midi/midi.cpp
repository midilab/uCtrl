#include "../../../../modules.h"

#ifdef USE_MIDI

#include "midi.hpp"

namespace uctrl { namespace module {

//
// multicore archs
//
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	portMUX_TYPE _uctrlMidiTimerMux = portMUX_INITIALIZER_UNLOCKED;
	#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uctrlMidiTimerMux); X; portEXIT_CRITICAL_ISR(&_uctrlMidiTimerMux);
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
}

Midi::~Midi()
{
	
}

void Midi::plug(midi::MidiInterface<midi::SerialMIDI<HardwareSerial> > * device)
{
	uint8_t port = _port_size;
	
	_midi_port_if[port] = device;
	
	_midi_port_if[port]->begin(MIDI_CHANNEL_OMNI);

	// Setup MIDI Callbacks to handle incomming messages
	_midi_port_if[port]->setHandleNoteOn(handleNoteOn);
	_midi_port_if[port]->setHandleNoteOff(handleNoteOff);
	_midi_port_if[port]->setHandleControlChange(handleCC);

	//_midi_port_if[port]->setHandleAfterTouchPoly(handleAfterTouchPoly);
	//_midi_port_if[port]->setHandleAfterTouchChannel(handleAfterTouchChannel);
	_midi_port_if[port]->setHandlePitchBend(handlePitchBend);
	_midi_port_if[port]->setHandleSystemExclusive(handleSystemExclusive);
		
	_midi_port_if[port]->setHandleClock(handleClock);
	_midi_port_if[port]->setHandleStart(handleStart);
	_midi_port_if[port]->setHandleStop(handleStop);
	_midi_port_if[port]->turnThruOff();	

	_port_size++;
}

#if defined(USE_USB_MIDI)
#if defined(TEENSYDUINO) && !defined(__AVR_ATmega32U4__)
void Midi::plug(usb_midi_class * device)
#elif defined(__AVR_ATmega32U4__)
void Midi::plug(midi::MidiInterface<usbMidi::usbMidiTransport> * device)
#endif
{
	if ( _usb_port_if == nullptr ) {
		
		_usb_port_if = device;
		_usb_port = _port_size;
		
		_usb_port_if->begin();

		// Setup MIDI Callbacks to handle incomming messages
		_usb_port_if->setHandleNoteOn(handleNoteOn);
		_usb_port_if->setHandleNoteOff(handleNoteOff);
		_usb_port_if->setHandleControlChange(handleCC);

		//_usb_port_if->setHandleAfterTouchPoly(handleAfterTouchPoly);
		//_usb_port_if->setHandleAfterTouchChannel(handleAfterTouchChannel);
#if defined(TEENSYDUINO) && !defined(__AVR_ATmega32U4__)
		_usb_port_if->setHandlePitchChange(handlePitchBend);
#elif defined(__AVR_ATmega32U4__)
		_usb_port_if->setHandlePitchBend(handlePitchBend);
#endif
		_usb_port_if->setHandleSystemExclusive(handleSystemExclusive);
			
		_usb_port_if->setHandleClock(handleClock);
		_usb_port_if->setHandleStart(handleStart);
		_usb_port_if->setHandleStop(handleStop);
		//device->turnThruOff();	

		_port_size++;
	}
}
#endif

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

// do only read if the port are not in realtime mode
bool Midi::read(uint8_t port)
{
	if ( port > _port_size || port == 0 ) {
		return false;
	}
	
	--port;
	
	_port = port;

#if (defined(TEENSYDUINO) || defined(__AVR_ATmega32U4__)) && defined(USE_USB_MIDI)
	if ( _usb_port == port ) {
		return _usb_port_if->read();
	}
#endif

	return _midi_port_if[port]->read();
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

#if (defined(TEENSYDUINO) || defined(__AVR_ATmega32U4__)) && defined(USE_USB_MIDI)
	if ( _usb_port == port ) {
		writeUsb(msg, interrupted);
		return;
	}
#endif

    // packages with interrupted 1 are marked to be handled with interruptions disable to avoid MIDI override data with MIDI messages sent via some timer interruption

    // MIDI Handle
    switch (msg->type) {

		//Realtime
		case uctrl::protocol::midi::Clock:
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendRealTime(midi::Clock))
			} else {
				_midi_port_if[port]->sendRealTime(midi::Clock);
			}
			break;   

		case uctrl::protocol::midi::Start:
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendRealTime(midi::Start))
			} else {
				_midi_port_if[port]->sendRealTime(midi::Start);
			}
			break;  

		case uctrl::protocol::midi::Stop:
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendRealTime(midi::Stop))
			} else {
				_midi_port_if[port]->sendRealTime(midi::Stop);
			}
			break;   

		// Non-realtime 
		case uctrl::protocol::midi::NoteOn:
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendNoteOn(msg->data1, msg->data2, msg->channel+1))
			} else {
				_midi_port_if[port]->sendNoteOn(msg->data1, msg->data2, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::NoteOff:
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendNoteOff(msg->data1, 0, msg->channel+1))
			} else {
				_midi_port_if[port]->sendNoteOff(msg->data1, 0, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::ControlChange:   
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendControlChange(msg->data1, msg->data2, msg->channel+1))
			} else {
				_midi_port_if[port]->sendControlChange(msg->data1, msg->data2, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::ProgramChange:
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendProgramChange(msg->data1, msg->channel+1))
			} else {
				_midi_port_if[port]->sendProgramChange(msg->data1, msg->channel+1);
			}
			break;
						
		case uctrl::protocol::midi::Nrpn:  
			if ( interrupted == 0 ) { 
				ATOMIC(
					// param select
					_midi_port_if[port]->sendControlChange(99, 0x7f & (msg->data1 >> 7), msg->channel+1);
					_midi_port_if[port]->sendControlChange(98, 0x7f & msg->data1, msg->channel+1);
					// send value
					_midi_port_if[port]->sendControlChange(6, 0x7f & (msg->data2 >> 7), msg->channel+1);
					_midi_port_if[port]->sendControlChange(38, 0x7f & msg->data2, msg->channel+1);
				)
			} else {
				// param select
				_midi_port_if[port]->sendControlChange(99, 0x7f & (msg->data1 >> 7), msg->channel+1);
				_midi_port_if[port]->sendControlChange(98, 0x7f & msg->data1, msg->channel+1);
				// send value
				_midi_port_if[port]->sendControlChange(6, 0x7f & (msg->data2 >> 7), msg->channel+1);
				_midi_port_if[port]->sendControlChange(38, 0x7f & msg->data2, msg->channel+1);
			}
			break;
		
		case uctrl::protocol::midi::PitchBend:
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendPitchBend(msg->data1, msg->channel+1))
			} else {
				//_midi_port_if[port]->sendPitchBend((int16_t)(((uint8_t)msg->data1 << 8 ) | ((uint8_t)msg->data2 & 0xff)), msg->channel+1);
				_midi_port_if[port]->sendPitchBend(msg->data1, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::Sysex:
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendSysEx(msg->data1, msg->sysex))
			} else {
				_midi_port_if[port]->sendSysEx(msg->data1, msg->sysex);
			}
			break;


		case uctrl::protocol::midi::AfterTouchPoly:
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendPolyPressure(msg->data1, msg->data2, msg->channel+1))
			} else {
				_midi_port_if[port]->sendPolyPressure(msg->data1, msg->data2, msg->channel+1);
			}
			break;
			
		case uctrl::protocol::midi::AfterTouchChannel:
			if ( interrupted == 0 ) { 
				ATOMIC(_midi_port_if[port]->sendAfterTouch(msg->data1, msg->channel+1))
			} else {
				_midi_port_if[port]->sendAfterTouch(msg->data1, msg->channel+1);
			}
			break;

		default:
			break;
    
    }
	
}

#if (defined(TEENSYDUINO) || defined(__AVR_ATmega32U4__)) && defined(USE_USB_MIDI)
void Midi::writeUsb(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t interrupted)
{
    // MIDI Handle
    switch (msg->type) {

		//Realtime
		case uctrl::protocol::midi::Clock:
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendRealTime(midi::Clock))
			} else {
				_usb_port_if->sendRealTime(midi::Clock);
				//_usb_port_if->send_now();
			}
			break;   

		case uctrl::protocol::midi::Start:
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendRealTime(midi::Start))
			} else {
				_usb_port_if->sendRealTime(midi::Start);
				//_usb_port_if->send_now();
			}
			break;  

		case uctrl::protocol::midi::Stop:
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendRealTime(midi::Stop))
			} else {
				_usb_port_if->sendRealTime(midi::Stop);
				//_usb_port_if->send_now();
			}
			break;   

		// Non-realtime 
		case uctrl::protocol::midi::NoteOn:
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendNoteOn(msg->data1, msg->data2, msg->channel+1))
			} else {
				_usb_port_if->sendNoteOn(msg->data1, msg->data2, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::NoteOff:
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendNoteOff(msg->data1, 0, msg->channel+1))
			} else {
				_usb_port_if->sendNoteOff(msg->data1, 0, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::ControlChange:   
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendControlChange(msg->data1, msg->data2, msg->channel+1))
			} else {
				_usb_port_if->sendControlChange(msg->data1, msg->data2, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::ProgramChange:
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendProgramChange(msg->data1, msg->channel+1))
			} else {
				_usb_port_if->sendProgramChange(msg->data1, msg->channel+1);
			}
			break;
						
		case uctrl::protocol::midi::Nrpn:  
			if ( interrupted == 0 ) { 
				ATOMIC(
					// param select
					_usb_port_if->sendControlChange(99, 0x7f & (msg->data1 >> 7), msg->channel+1);
					_usb_port_if->sendControlChange(98, 0x7f & msg->data1, msg->channel+1);
					// send value
					_usb_port_if->sendControlChange(6, 0x7f & (msg->data2 >> 7), msg->channel+1);
					_usb_port_if->sendControlChange(38, 0x7f & msg->data2, msg->channel+1);
				)
			} else {
				// param select
				_usb_port_if->sendControlChange(99, 0x7f & (msg->data1 >> 7), msg->channel+1);
				_usb_port_if->sendControlChange(98, 0x7f & msg->data1, msg->channel+1);
				// send value
				_usb_port_if->sendControlChange(6, 0x7f & (msg->data2 >> 7), msg->channel+1);
				_usb_port_if->sendControlChange(38, 0x7f & msg->data2, msg->channel+1);
			}   
			break;
		
		case uctrl::protocol::midi::PitchBend:
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendPitchBend(msg->data1, msg->channel+1))
			} else {
				_usb_port_if->sendPitchBend(msg->data1, msg->channel+1);
			}
			break;

		case uctrl::protocol::midi::Sysex:
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendSysEx(msg->data1, msg->sysex))
			} else {
				_usb_port_if->sendSysEx(msg->data1, msg->sysex);
			}
			break;

		case uctrl::protocol::midi::AfterTouchPoly:
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendPolyPressure(msg->data1, msg->data2, msg->channel+1))
			} else {
				_usb_port_if->sendPolyPressure(msg->data1, msg->data2, msg->channel+1);
			}
			break;
			
		case uctrl::protocol::midi::AfterTouchChannel:
			if ( interrupted == 0 ) { 
				ATOMIC(_usb_port_if->sendAfterTouch(msg->data1, msg->channel+1))
			} else {
				_usb_port_if->sendAfterTouch(msg->data1, msg->channel+1);
			}
			break;
    }
    
}
#endif

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
