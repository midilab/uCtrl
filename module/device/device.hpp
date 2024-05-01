#ifndef __U_CTRL_DEVICE_HPP__
#define __U_CTRL_DEVICE_HPP__

#include <Arduino.h>
#include "device.h"

namespace uctrl { namespace module { 

class Device
{
    public:
        Device();
        ~Device(); 

    void init(uint8_t device_number, uint16_t event_buffer_size, uint8_t sysex_buffer_size = 0, uint16_t device_label_buffer_size = 0);

	bool isPressed(uint8_t port);
	
	void lockControl(uint8_t remote_port);

	//
	// Import and Export memory data
	//
	//CONTROL_DATA getAnalogData();
	//loadAnalogData(CONTROL_DATA);
	//dumpDigitalData();
	//loadDigitalData();

	//
	// CALLBACK stuffs
	//
	// App ctrl event callback
	void (*_appCtrlDataCallback)(uint16_t control_id, uint16_t value) = nullptr;
	void setAppCtrlCallback(void (*callback)(uint16_t control_id, uint16_t value)) {
		_appCtrlDataCallback = callback;
	}
	uint16_t _adc_max_resolution = 1024;
	
	//
	// Processors
	//
    bool handleAnalogEvent(uint8_t port, uint16_t value, uint8_t interrupted);
    bool handleDigitalEvent(uint8_t port, uint16_t value, uint8_t interrupted);
	void remoteEventHandler(CONTROL_DATA * event, uint8_t device_id,  uint16_t value, bool chain_mode, uint8_t interrupted);

    // edit ctrl
    void setupCtrl(uint8_t port, uint16_t value);

    //uctrl::uCtrlClass * _uCtrl;

	//
	// Attributes definitions 
	//
	// Main data structure
	REMOTE_DATA _remote;
	// The buffer area for events data
	CONTROL_DATA * _remote_event_buffer;
	uint16_t _remote_event_buffer_size = 0;
	uint8_t _device_buffer_size = 0;
	// The buffer area for sysex template data
	MIDI_SYSEX_TEMPLATE * _midi_sysex_template_buffer;
	uint8_t _midi_sysex_template_buffer_size = 0;
	
	uint8_t _selected_scene;
	
	bool _data_feedback_show = false;
	
	uint8_t _remote_scene_number;
	
	uint8_t _event_id_filter;

	uint8_t _subpage;

	uint8_t _edit_ctrl_selected_port = 0;
	uint16_t _edit_ctrl_last_value = 0;

	//uint32_t _delay_control;

	// Devices data
	//uint8_t _buffer_layout_counter = 0;
	DEVICE_DATA * _device;
	uint8_t _device_number = 0;
	CONTROL_DATA _event_data_buffer[2];
	//uint16_t _cached_event[2] = { 65535, 65535 };
	uint8_t _selected_device = 0;
	uint8_t _selected_control_mode;
	uint16_t _device_event_size = 0;

	// Devices Memory managment calls
	void clearMap(uint8_t device_id);
	void eventMap(uint8_t device_id, CONTROL_DATA * event, uint8_t * eventName = nullptr, int16_t label_offset = 0);
	void configDevice(uint8_t device_id, uint8_t port, uint8_t channel, uint8_t * deviceName = nullptr);
	void selectDevice(uint8_t selected_device);
	void adcMap(uint8_t port, uint8_t device_id, uint16_t controllerAddress);
	void dinMap(uint8_t port, uint8_t device_id, uint16_t controllerAddress, uint8_t button_behaviour = 0, int16_t increment_base = 0);
	void adcMapClear(uint8_t port, uint8_t mode = 0);
	void dinMapClear(uint8_t port, uint8_t mode = 0);
	int8_t getAdcDeviceMode(uint8_t port);
	uint8_t getAdcCtrlNumber(uint8_t port);
	uint8_t getAdcDeviceNumber(uint8_t port);
	int8_t getDinDeviceMode(uint8_t port);
	void setDevicePort(uint8_t port, uint8_t device_id = 0);
	void setDeviceChannel(uint8_t channel, uint8_t device_id = 0);
	uint8_t getDevicePort(uint8_t device_id = 0);
	uint8_t getDeviceChannel(uint8_t device_id = 0);

#ifndef LOW_RESOURCE_MCU	
	uint8_t * getDeviceName(uint8_t device_id = 0);
	uint8_t getDinCtrlBehaviour(uint8_t port);
	int16_t getDinCtrlIncrement(uint8_t port);	
#endif

	uint8_t getDinCtrlNumber(uint8_t port);
	uint8_t getDinDeviceNumber(uint8_t port);	
	
	//uint8_t getDeviceMapBufferSize();
	void getDeviceMapEventData(uint8_t device_id, uint16_t event_address, CONTROL_DATA * event);
	uint8_t getMaxDeviceNumber();
	//void setDevice(uint8_t device_id);
	void getEvent(uint16_t address, CONTROL_DATA * event);
	//void sendEvent(uint8_t device_id, uint8_t eventId, uint16_t value, uint16_t min, uint16_t max);
	//void sendEvent(uint16_t eventAddress, uint8_t device_id, uint16_t value);
	void ctrlMap(uint8_t device_id, uint16_t control_id, uint16_t range_min, uint16_t range_max, int16_t label_offset = 0, uint8_t * name = NULL);
	
	void editCtrlsetDevice(int8_t inc, uint8_t port = 0);
    void editCtrlsetCtrl(int8_t inc, uint8_t port = 0);	
	int16_t _last_buffer_address = 1023;
    
    void setCtrlMode(uint8_t mode);
    uint8_t getCtrlMode();
    uint8_t _ctrl_mode = 0;

//#ifdef USE_MIDI
	// start device stuffs
	uint8_t rolandChecksum(uint8_t * sysex_buffer, uint8_t buffer_size);
	void sysexTemplateRegister(MIDI_SYSEX_TEMPLATE * sysex_template, uint8_t id);
	void midiMap(uint8_t device_id, uctrl::protocol::midi::MIDI_MESSAGE msg, uint16_t min = 0, uint16_t max = 127, int16_t label_offset = 0, uint8_t * eventName = NULL, uint8_t config = 0);
	uctrl::protocol::midi::MIDI_MESSAGE * eventToMidiMsg(CONTROL_DATA * event, uint16_t value, uint8_t channel);
	CONTROL_DATA * midiMsgToEvent(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint16_t min_value_range, uint16_t max_value_range, CONTROL_DATA * event, uint8_t config = 0);
    uint8_t _sysex[12];	
    // end device stuffs
//#endif


#ifdef USE_DEVICE_LABELS
	// part of device module
	void (*_controlDataFeedbackCallback)(uctrl::CONTROL_DATA_INFO * ctrl, uint8_t edit_mode) = nullptr;
	void setControlDataFeedbackCallback(void (*callback)(uctrl::CONTROL_DATA_INFO * ctrl, uint8_t edit_mode)) {
		_controlDataFeedbackCallback = callback;
	}	
	void setDataFeedbackTimeout(uint16_t milliseconds);
	void setSharedDataLabel(uint8_t device_id);
	void setDataLabel(const uint8_t * label, uint16_t range, uint8_t device_id);
	const uint8_t * getDataLabel(uint8_t device_id, uint16_t address, uint16_t value);
	uint16_t _label_buffer_size = 0;
    void dataFeedbackCallback();	
    bool showDataFeedback();
    void setDataFeedback(bool status);
    uint32_t getDataFeedbackTimeout();
	uint32_t _data_feedback_timeout_timer = 0;
	uint8_t _lc1024_data_label_buffer_id;
	uint16_t _data_labels_buffer_size = 0;
	uint16_t _shared_datalabel_address = 65535;    
#endif

    uint16_t getCtrlAdcMin(uint8_t port);
    uint16_t getCtrlAdcMax(uint8_t port);

	CONTROL_DATA_INFO _ctrl;
//#endif 
//#endif
	
	uint8_t _lc1024_buffer_id;
    
};

} }

//extern uctrl::module::Device device_module;

#endif
