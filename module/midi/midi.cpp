#include "midi.hpp"

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    SemaphoreHandle_t _midi_mutex;
#endif

namespace uctrl { namespace module {

Midi::Midi()
{
	_midiInputCallback = nullptr;
	_midiOutputCallback = nullptr;
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	_midi_mutex = xSemaphoreCreateMutex();
#endif
}

Midi::~Midi()
{
	
}

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
bool Midi::read(uint8_t port, uint8_t interrupted)
{
	if ( port > _port_size || port == 0 ) {
		return false;
	}
	
	--port;
	
	_port = port;

	// Use the stored function pointers to invoke member functions
	// should always be atomic interrupted=0
	//readFunctions[_port](midiArray[_port], interrupted);
	//midiArray[_port]->read(interrupted);
	midiArray[_port]->read(0);
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

void Midi::writeMidiInterface(uint8_t port, uctrl::protocol::midi::MIDI_MESSAGE * msg, uint8_t interrupted) {
    // packages with interrupted 1 are marked to be handled with interruptions disable to avoid MIDI override data with MIDI messages sent via some timer interruption
	
    // MIDI Handle
    switch (msg->type) {

		//Realtime
		case uctrl::protocol::midi::Clock:
			//sendFunctions[port](midiArray[port], midi::Clock, 0, 0, 0, interrupted);
			midiArray[port]->send(midi::Clock, 0, 0, 0, interrupted);
			break;   

		case uctrl::protocol::midi::Start:
			//sendFunctions[port](midiArray[port], midi::Start, 0, 0, 0, interrupted);
			midiArray[port]->send(midi::Start, 0, 0, 0, interrupted);
			break;  

		case uctrl::protocol::midi::Stop:
			//device->sendRealTime(midi::Stop);
			//sendFunctions[port](midiArray[port], midi::Stop, 0, 0, 0, interrupted);
			midiArray[port]->send(midi::Stop, 0, 0, 0, interrupted);
			break;   

		// Non-realtime 
		case uctrl::protocol::midi::NoteOn:
			//device->sendNoteOn(msg->data1, msg->data2, msg->channel+1);
			//sendFunctions[port](midiArray[port], midi::NoteOn, msg->data1, msg->data2, msg->channel+1, interrupted);
			midiArray[port]->send(midi::NoteOn, msg->data1, msg->data2, msg->channel+1, interrupted);
			break;

		case uctrl::protocol::midi::NoteOff:
			//device->sendNoteOff(msg->data1, 0, msg->channel+1);
			//sendFunctions[port](midiArray[port], midi::NoteOff, msg->data1, 0, msg->channel+1, interrupted);
			midiArray[port]->send(midi::NoteOff, msg->data1, 0, msg->channel+1, interrupted);
			break;

		case uctrl::protocol::midi::ControlChange:   
			//device->sendControlChange(msg->data1, msg->data2, msg->channel+1);
			//sendFunctions[port](midiArray[port], midi::ControlChange, msg->data1, msg->data2, msg->channel+1, interrupted);
			midiArray[port]->send(midi::ControlChange, msg->data1, msg->data2, msg->channel+1, interrupted);
			break;

		case uctrl::protocol::midi::ProgramChange:
			//device->sendProgramChange(msg->data1, msg->channel+1);
			//sendFunctions[port](midiArray[port], midi::ProgramChange, msg->data1, msg->data2, msg->channel+1, interrupted);
			midiArray[port]->send(midi::ProgramChange, msg->data1, msg->data2, msg->channel+1, interrupted);
			break;
						
		case uctrl::protocol::midi::Nrpn:  
			// param select
			//device->sendControlChange(99, 0x7f & (msg->data1 >> 7), msg->channel+1);
			//device->sendControlChange(98, 0x7f & msg->data1, msg->channel+1);
			//sendFunctions[port](midiArray[port], midi::ControlChange, 99, 0x7f & (msg->data1 >> 7), msg->channel+1, interrupted);
			//sendFunctions[port](midiArray[port], midi::ControlChange, 98, 0x7f & msg->data1, msg->channel+1, interrupted);
			midiArray[port]->send(midi::ControlChange, 99, 0x7f & (msg->data1 >> 7), msg->channel+1, interrupted);
			midiArray[port]->send(midi::ControlChange, 98, 0x7f & msg->data1, msg->channel+1, interrupted);
			// send value
			//device->sendControlChange(6, 0x7f & (msg->data2 >> 7), msg->channel+1);
			//device->sendControlChange(38, 0x7f & msg->data2, msg->channel+1);
			//sendFunctions[port](midiArray[port], midi::ControlChange, 6, 0x7f & (msg->data2 >> 7), msg->channel+1, interrupted);
			//sendFunctions[port](midiArray[port], midi::ControlChange, 38, 0x7f & msg->data2, msg->channel+1, interrupted);
			midiArray[port]->send(midi::ControlChange, 6, 0x7f & (msg->data2 >> 7), msg->channel+1, interrupted);
			midiArray[port]->send(midi::ControlChange, 38, 0x7f & msg->data2, msg->channel+1, interrupted);
			break;
		/* 
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
 */
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

	writeMidiInterface(port, msg, interrupted);
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

void Midi::handleClock(void)
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.type = uctrl::protocol::midi::Clock;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}	
}

void Midi::handleStart(void)
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.type = uctrl::protocol::midi::Start;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}	
}

void Midi::handleStop(void)
{
	if ( midi_module._midiInputCallback != nullptr ) {
		midi_module._message.type = uctrl::protocol::midi::Stop;
		midi_module._midiInputCallback(&midi_module._message, midi_module._port+1, 0);
	}	
}

} }

uctrl::module::Midi midi_module;