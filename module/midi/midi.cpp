#include "midi.hpp"

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

/* template <typename T, typename U, typename V>
void Midi::readImpl(void *item, uint8_t interrupted) {
	midi::MidiInterface<T, U, V> *midi = reinterpret_cast<midi::MidiInterface<T, U, V> *>(item);
	midi->read();
}

template <typename T, typename U, typename V>
void Midi::sendImpl(void *item, const midi::MidiType &inType, const midi::DataByte &inData1,
					const midi::DataByte &inData2, const midi::Channel &inChannel, uint8_t interrupted) {
	midi::MidiInterface<T, U, V> *midi = reinterpret_cast<midi::MidiInterface<T, U, V> *>(item);
	if (interrupted == 0) {
		ATOMIC(midi->send(inType, inData1, inData2, inChannel));
	} else {
		midi->send(inType, inData1, inData2, inChannel);
	}
} */
/* 
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
 */

// Method to store a MidiInterface object and keep track of it by index
//template <typename T, typename U>
//void Midi::plug(midi::MidiInterface<T, U>& midiInterface) {

/* 
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

void onConnected() {

}

void OnDisconnected() {

}

#if defined(USE_BT_MIDI_ESP32) && defined(CONFIG_BT_ENABLED) && (defined(ARDUINO_ARCH_ESP32) || defined(ESP32))
void Midi::plug(midi::MidiInterface<bleMidi::BLEMIDI_Transport<bleMidi::BLEMIDI_ESP32>, bleMidi::MySettings> * device)
{
	if ( _ble_port == 255 ) {
		_ble_port = _port_size;
		_midi_port_if[_port_size] = (void *) device;

		initMidiInterface<midi::MidiInterface<bleMidi::BLEMIDI_Transport<bleMidi::BLEMIDI_ESP32>, bleMidi::MySettings>>((midi::MidiInterface<bleMidi::BLEMIDI_Transport<bleMidi::BLEMIDI_ESP32>, bleMidi::MySettings> *)_midi_port_if[_port_size]);

		device->setHandlePitchBend(handlePitchBend);
		device->turnThruOff();	

		// how to handle this for API access?
		//BLEMIDI.setHandleConnected(OnConnected); // void OnConnected() {}
		//BLEMIDI.setHandleDisconnected(OnDisconnected); // void OnDisconnected() {}
		_port_size++;
	}
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

// do only read if the port are not in realtime mode
bool Midi::read(uint8_t port)
{
	if ( port > _port_size || port == 0 ) {
		return false;
	}
	
	--port;
	
	_port = port;
	
	uint8_t interrupted = 1;
	// Use the stored function pointers to invoke member functions
	readFunctions[_port](midiArray[_port], interrupted);
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
			sendFunctions[port](midiArray[port], midi::Clock, 0, 0, 0, interrupted);
			break;   

		case uctrl::protocol::midi::Start:
			sendFunctions[port](midiArray[port], midi::Start, 0, 0, 0, interrupted);
			break;  

		case uctrl::protocol::midi::Stop:
			//device->sendRealTime(midi::Stop);
			sendFunctions[port](midiArray[port], midi::Stop, 0, 0, 0, interrupted);
			break;   

		// Non-realtime 
		case uctrl::protocol::midi::NoteOn:
			//device->sendNoteOn(msg->data1, msg->data2, msg->channel+1);
			sendFunctions[port](midiArray[port], midi::NoteOn, msg->data1, msg->data2, msg->channel+1, interrupted);
			break;

		case uctrl::protocol::midi::NoteOff:
			//device->sendNoteOff(msg->data1, 0, msg->channel+1);
			sendFunctions[port](midiArray[port], midi::NoteOff, msg->data1, 0, msg->channel+1, interrupted);
			break;

		case uctrl::protocol::midi::ControlChange:   
			//device->sendControlChange(msg->data1, msg->data2, msg->channel+1);
			sendFunctions[port](midiArray[port], midi::ControlChange, msg->data1, msg->data2, msg->channel+1, interrupted);
			break;

		case uctrl::protocol::midi::ProgramChange:
			//device->sendProgramChange(msg->data1, msg->channel+1);
			sendFunctions[port](midiArray[port], midi::ProgramChange, msg->data1, msg->data2, msg->channel+1, interrupted);
			break;
						
		case uctrl::protocol::midi::Nrpn:  
			// param select
			//device->sendControlChange(99, 0x7f & (msg->data1 >> 7), msg->channel+1);
			//device->sendControlChange(98, 0x7f & msg->data1, msg->channel+1);
			sendFunctions[port](midiArray[port], midi::ControlChange, 99, 0x7f & (msg->data1 >> 7), msg->channel+1, interrupted);
			sendFunctions[port](midiArray[port], midi::ControlChange, 98, 0x7f & msg->data1, msg->channel+1, interrupted);
			// send value
			//device->sendControlChange(6, 0x7f & (msg->data2 >> 7), msg->channel+1);
			//device->sendControlChange(38, 0x7f & msg->data2, msg->channel+1);
			sendFunctions[port](midiArray[port], midi::ControlChange, 6, 0x7f & (msg->data2 >> 7), msg->channel+1, interrupted);
			sendFunctions[port](midiArray[port], midi::ControlChange, 38, 0x7f & msg->data2, msg->channel+1, interrupted);
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

	// writing ble crashes! why? only inside rtos task, when interrupted == 1
	if (interrupted == 1) return;
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