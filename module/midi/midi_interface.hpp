#ifndef __U_CTRL_MIDI_INTERFACE_HPP__
#define __U_CTRL_MIDI_INTERFACE_HPP__

#include "midi.h"
#include "midi.hpp"

class BaseMidiInterface {
public:
    virtual ~BaseMidiInterface() {}
    virtual void read(uint8_t interrupted) = 0;
    virtual void send(const midi::MidiType& inType, const midi::DataByte& inData1,
                    const midi::DataByte& inData2, const midi::Channel& inChannel,
                    uint8_t interrupted = 0) = 0;
    virtual void sendSysEx(uctrl::protocol::midi::MIDI_MESSAGE* msg, uint8_t interrupted);
};

template <typename T>
class MidiInterfaceWrapper : public BaseMidiInterface {
public:
    MidiInterfaceWrapper(T* midiInterface) : _midiInterface(midiInterface) {}

    // Replace static_cast with direct access to _midiInterface
    void read(uint8_t interrupted = 0) {
        if (interrupted == 0) {
            MIDI_ATOMIC(_midiInterface->read());
        } else {
            _midiInterface->read();
        }
    }

    void send(const midi::MidiType& inType, const midi::DataByte& inData1,
            const midi::DataByte& inData2, const midi::Channel& inChannel,
            uint8_t interrupted) {
        if (interrupted == 0) {
            MIDI_ATOMIC(_midiInterface->send(inType, inData1, inData2, inChannel));
        } else {
            _midiInterface->send(inType, inData1, inData2, inChannel);
        }
    }

    void sendSysEx(uctrl::protocol::midi::MIDI_MESSAGE* msg, uint8_t interrupted) {
        if (interrupted == 0) {
            MIDI_ATOMIC(_midiInterface->sendSysEx(msg->data1, msg->sysex));
        } else {
            _midiInterface->sendSysEx(msg->data1, msg->sysex);
        }
    }

private:
    T* _midiInterface;
};


#if defined(TEENSYDUINO) && defined(USB_MIDI_SERIAL) && !defined(__AVR_ATmega32U4__)
template <typename T>
class MidiInterfaceWrapperTeensy : public BaseMidiInterface {
public:
    MidiInterfaceWrapperTeensy(T* midiInterface) : _midiInterface(midiInterface) {}

    // Replace static_cast with direct access to _midiInterface
    void read(uint8_t interrupted = 0) {
        if (interrupted == 0) {
            MIDI_ATOMIC(_midiInterface->read());
        } else {
            _midiInterface->read();
        }
    }

    void send(const midi::MidiType& inType, const midi::DataByte& inData1,
            const midi::DataByte& inData2, const midi::Channel& inChannel,
            uint8_t interrupted) {
        if (interrupted == 0) {
            MIDI_ATOMIC(_midiInterface->send(inType, inData1, inData2, inChannel, 0));
        } else {
            _midiInterface->send(inType, inData1, inData2, inChannel, 0);
        }
    }

    void sendSysEx(uctrl::protocol::midi::MIDI_MESSAGE* msg, uint8_t interrupted) {
        if (interrupted == 0) {
            MIDI_ATOMIC(_midiInterface->sendSysEx(msg->data1, msg->sysex, 0));
        } else {
            _midiInterface->sendSysEx(msg->data1, msg->sysex, 0);
        }
    }

private:
    T* _midiInterface;
};
#endif

#endif
