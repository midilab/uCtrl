#include "../../uCtrl.h"
#include "device.hpp"

namespace uctrl { namespace module {

Device::Device()
{

}

Device::~Device()
{
	//
	// Destructor
	// 1) Let the heap be free!
	
	// Free all remote analog ports
	free(_remote.adc_port);
	
	// Free all remote analog ports
	free(_remote.din_port);

//#if !defined(UMODULAR_DRIVER_LC1024)
	// Free remote event buffer
	free(_remote_event_buffer);
//#endif 
}

// get uCtrl reference to handle outside modules? or try to do it inside uCtrl?
//void Device::init(uctrl::uCtrlClass * uCtrl, uint8_t device_number, uint16_t event_buffer_size, uint8_t device_buffer_size, uint16_t device_label_buffer_size)
void Device::init(uint8_t device_number, uint16_t event_buffer_size, uint8_t sysex_buffer_size, uint16_t device_label_buffer_size)
{
	uint8_t port_size;
    
	_device_number = device_number;
	_remote_event_buffer_size = event_buffer_size;
	_midi_sysex_template_buffer_size = sysex_buffer_size;

	// global reference for uCtrl to access modules
	//_uCtrl = uCtrl;
#ifdef USE_DEVICE_LABELS
	_data_labels_buffer_size = device_label_buffer_size;
#endif
    
	// 
	// initing the uCtrl
	//
	// Allocate memory for control data
	//
	// Memory allocation rules
	//
	// 1) Alloc it once and forever!
	// 2) Manage the buffer by our own
	
	// code for device module
#ifdef USE_EXT_RAM   
	
	_lc1024_buffer_id = uCtrl.ram->registerBuffer( sizeof(CONTROL_DATA), _remote_event_buffer_size );

	#ifdef USE_DEVICE_LABELS
	if ( uCtrl.oled != nullptr ) {
		_lc1024_data_label_buffer_id = uCtrl.ram->registerBuffer( sizeof(DATA_LABEL), _data_labels_buffer_size );
	}
	_label_buffer_size = floor(_data_labels_buffer_size / _device_number);
	#endif	

#else
	
	// Alloc the remote event data memory buffer
	_remote_event_buffer = (CONTROL_DATA*) malloc( sizeof(CONTROL_DATA) * _remote_event_buffer_size );

	// Initing _remote_event_buffer
	for (uint16_t loc = 0; loc < _remote_event_buffer_size; loc++) {
		_remote_event_buffer[loc].type = DISABLE;
		//_remote_event_buffer[loc].next = NULL;
	}
	
	#ifdef USE_DEVICE_LABELS
	// change that!
	if ( uCtrl.oled != nullptr ) {
		_lc1024_data_label_buffer_id = uCtrl.ram->registerBuffer( sizeof(DATA_LABEL), _data_labels_buffer_size );
	}
	_label_buffer_size = floor(_data_labels_buffer_size / _device_number);
	#endif

#endif		

	// Alloc device(s) memory
	_device = (DEVICE_DATA*) malloc( sizeof(DEVICE_DATA) * _device_number );

	_device_event_size = floor(_remote_event_buffer_size / _device_number);

	// initing devices structure
	for ( uint8_t i = 0; i < _device_number; i++ ) {
		_device[i].active = 0;
		_device[i].port = 0;
		_device[i].chn = 0;
		_device[i].buffer_address = _device_event_size * i;
		_device[i].buffer_size = 0;		
#ifdef USE_DEVICE_LABELS
		_device[i].label_buffer_address = _label_buffer_size * i;
		_device[i].label_buffer_size = 0;
#endif	
	}

	// Alloc ports memory pointers
	// Each port costs 2 bytes on arduino platform(the size of a memory pointer)  
	// ADC ports    
	port_size = uCtrl.getAnalogPorts();
	if ( port_size > 0 ) {

		_remote.adc_port = (ADC_PORT_DATA*) malloc( sizeof(ADC_PORT_DATA) * port_size );
		
		// Initing ports to NULL
		for ( uint8_t i=0; i < port_size; i++ ) {
			_remote.adc_port[i].event_address = 1023; //-1;
			_remote.adc_port[i].device_id = 0;
			_remote.adc_port[i].mode = 0;
			_remote.adc_port[i].min = 0;
			//_remote.adc_port[i].max = 1023;
			_remote.adc_port[i].max = 127;
		}
		
	}

	// Digital ports
	port_size = uCtrl.getDigitalPorts();
	if ( port_size > 0 ) {

		_remote.din_port = (DIN_PORT_DATA*) malloc( sizeof(DIN_PORT_DATA) * port_size );
		
		// Initing ports(s) to NULL
		for ( uint8_t i=0; i < port_size; i++ ) {
			_remote.din_port[i].event_address = 1023; //-1;
			_remote.din_port[i].device_id = 0;
			_remote.din_port[i].mode = 0;
#if defined(USE_DEVICE_LABELS)			
			_remote.din_port[i].value = 0;
			_remote.din_port[i].behaviour = 0;
			_remote.din_port[i].increment_base = 0;	
#endif		
		}
		
	}
	
	if ( _midi_sysex_template_buffer_size > 0 ) {
		// Alloc sysex template memory area
		_midi_sysex_template_buffer = (MIDI_SYSEX_TEMPLATE*) malloc( sizeof(MIDI_SYSEX_TEMPLATE) * _midi_sysex_template_buffer_size );

		// Initing _midi_sysex_template_buffer
		for (uint8_t loc = 0; loc < _midi_sysex_template_buffer_size; loc++) {
			_midi_sysex_template_buffer[loc].size = 0; // zero means not allocated
			//_midi_sysex_template_buffer[loc].data = 0;
		}
	}	    
        
}

// interrupted calls will process only the essential realtime requests, leaving others to be processed outside ISR()
bool Device::handleAnalogEvent(uint8_t port, uint16_t value, uint8_t interrupted)
{
	//static uint16_t address;
	
    //--port;
    if ( _remote.adc_port[port].event_address != 1023 ) { //>= 0 ) {
        // Handler dispatcher
        _ctrl.port = port+1;
		_ctrl.device_id = _remote.adc_port[port].device_id+1;
		// check the last event buffer request, in case it cached, return the cache instead of accessing the memory again
		// its cached?
		//address = _device[_remote.adc_port[port].device_id].buffer_address + _remote.adc_port[port].event_address;
		//if ( _cached_event[interrupted] != address ) {
			getDeviceMapEventData(_remote.adc_port[port].device_id, _remote.adc_port[port].event_address, &_event_data_buffer[interrupted]);
			//_cached_event[interrupted] = address;
		//}
        remoteEventHandler(&_event_data_buffer[interrupted], _remote.adc_port[port].device_id, value, true, interrupted);
        return true;
    } else {
        return false;
    }   
}

bool Device::handleDigitalEvent(uint8_t port, uint16_t value, uint8_t interrupted)
{
    //--port;
    if ( _remote.din_port[port].event_address != 1023 ) { //!= -1 ) {  
        //_ctrl.port = port+1;
#if defined(USE_DEVICE_LABELS)
        switch (_remote.din_port[port].behaviour) {	
            
            // PRESS WITH INCREMENT?
            case 0:
                if ( value > 0 ) { // on press! no release action here
                    
                    _remote.din_port[port].value += _remote.din_port[port].increment_base;

                    if ( _remote.din_port[port].value < 0 ) {
                         _remote.din_port[port].value = _adc_max_resolution;
                    } else if ( _remote.din_port[port].value > _adc_max_resolution ) {
                         _remote.din_port[port].value = 0;
                    }
                    
                    value = (_remote.din_port[port].value / (_adc_max_resolution / abs( _remote.din_port[port].max - _remote.din_port[port].min )) + _remote.din_port[port].min);
                    
                    // Handler dispatcher
                    getDeviceMapEventData(_remote.din_port[port].device_id, (uint16_t)_remote.din_port[port].event_address, &_event_data_buffer[interrupted]);
                    remoteEventHandler(&_event_data_buffer[interrupted], _remote.din_port[port].device_id, value, true, interrupted);
                }
                break;
            case 1:
                // pressed and not pressed
                // Handler dispatcher
                getDeviceMapEventData(_remote.din_port[port].device_id, (uint16_t)_remote.din_port[port].event_address, &_event_data_buffer[interrupted]);
                remoteEventHandler(&_event_data_buffer[interrupted], _remote.din_port[port].device_id, value, true, interrupted);
                break;
        }
#else
        // Handler dispatcher
        getDeviceMapEventData(_remote.din_port[port].device_id, (uint16_t)_remote.din_port[port].event_address, &_event_data_buffer[interrupted]);
        remoteEventHandler(&_event_data_buffer[interrupted], _remote.din_port[port].device_id, value, true, interrupted);
#endif
        return true;
    } else {
        return false;
    }
}

void Device::setupCtrl(uint8_t port, uint16_t value)
{
	uint8_t size_of_ports;
	//--port;

	// 1023 default value we got no _remote.adc_port[port].device_id
	if ( _remote.adc_port[port].event_address == 1023 ) {
		noInterrupts();
	    //_remote.adc_port[port].event_address = _device[_selected_device].buffer_address;
		_remote.adc_port[port].event_address = _last_buffer_address != 1023 ? _last_buffer_address : _device[_selected_device].buffer_address;
	    _remote.adc_port[port].device_id = _selected_device;
	    _remote.adc_port[port].mode = 0;
	    _remote.adc_port[port].min = 0;
	    _remote.adc_port[port].max = 127;
	    interrupts();
	}
	noInterrupts();
	_edit_ctrl_selected_port = port;
	_edit_ctrl_last_value = value;
	interrupts();
	size_of_ports = uCtrl.getAnalogPorts();
	// lock all controllers
	for ( uint8_t lock=0; lock < size_of_ports; lock++ ) {
	    // but do not lock our in edit controller
	    if ( lock != _edit_ctrl_selected_port ) {
			lockControl(lock+1);
	    }
	}
}

void Device::selectDevice(uint8_t selected_device) 
{
	if ( selected_device > _device_number ) {
		return;
	}		
	selected_device--;
	noInterrupts();
	_selected_device = selected_device;
	interrupts();
}

uint8_t Device::getMaxDeviceNumber()
{
	return _device_number;
}

void Device::clearMap(uint8_t device_id)
{
	if ( device_id > _device_number ) {
		return;
	}		
	device_id--;
	_device[device_id].buffer_size = 0;
#if defined(USE_DEVICE_LABELS)
	_device[device_id].label_buffer_size = 0;
#endif
}

void Device::eventMap(uint8_t device_id, CONTROL_DATA * event, uint8_t * eventName, int16_t label_offset)
{
	uint16_t memory_address;
	
	if ( device_id > _device_number ) {
		return;
	}	
	device_id--;
	
	if ( _device[device_id].buffer_size >= _device_event_size ) {
		return; // full buffer!
	}
	
	memory_address = _device[device_id].buffer_address + _device[device_id].buffer_size;

#ifdef USE_DEVICE_LABELS  
	//memcpy (_event_data_buffer.name, eventName, 16);
	memset(event->name,' ',16);
	for ( uint8_t i=0; i < 16; i++ ) {
		if ( eventName[i] == '\0' ) {
		    break;
		}
		event->name[i] = eventName[i];
	}
	event->data_label = 65535;
	event->label_offset = label_offset;
#endif
#ifdef USE_EXT_RAM 
	uCtrl.ram->write((uint8_t*)event, memory_address, _lc1024_buffer_id);
#else
	_remote_event_buffer[memory_address] = *event;
#endif
	_device[device_id].buffer_size++;
}

int8_t Device::getAdcDeviceMode(uint8_t port)
{
	port--;
	if ( _remote.adc_port[port].event_address != 1023 ) {
		return _remote.adc_port[port].mode;
	} else {
		return -1;
	}
}

uint8_t Device::getAdcCtrlNumber(uint8_t port)
{
	port--;
	return _remote.adc_port[port].event_address+1;
}

uint8_t Device::getAdcDeviceNumber(uint8_t port)
{
	port--;
	return _remote.adc_port[port].device_id+1;
}

int8_t Device::getDinDeviceMode(uint8_t port)
{
	port--;
	if ( _remote.din_port[port].event_address != 1023 ) {
		return _remote.din_port[port].mode;
	} else {
		return -1;
	}
}

void Device::setDevicePort(uint8_t port, uint8_t device_id)
{
	if ( device_id == 0 ) {
		device_id = _selected_device;
	}	
	noInterrupts();
	_device[device_id].port = port;
	interrupts();
}

void Device::setDeviceChannel(uint8_t channel, uint8_t device_id)
{
	if ( device_id == 0 ) {
		device_id = _selected_device;
	}
	noInterrupts();
	_device[device_id].chn = channel;
	interrupts();
}

uint8_t Device::getDevicePort(uint8_t device_id)
{
	if ( device_id == 0 ) {
		device_id = _selected_device;
	}	
	return _device[device_id].port;
}

uint8_t Device::getDeviceChannel(uint8_t device_id)
{
	if ( device_id == 0 ) {
		device_id = _selected_device;
	}
	return _device[device_id].chn;
}

uint8_t Device::getDinCtrlNumber(uint8_t port)
{
	port--;
	return _remote.din_port[port].event_address+1;
}

uint8_t Device::getDinDeviceNumber(uint8_t port)
{
	port--;
	return _remote.din_port[port].device_id+1;
}

void Device::getEvent(uint16_t address, CONTROL_DATA * event_buffer)
{	
#ifdef USE_EXT_RAM   
	uCtrl.ram->read((uint8_t*)event_buffer, address, _lc1024_buffer_id);
#else
	*event_buffer = _remote_event_buffer[address];
#endif
}

// public access to get general info about a controller data. generally accessed from UI
void Device::getDeviceMapEventData(uint8_t device_id, uint16_t event_address, CONTROL_DATA * event_buffer)
{
	getEvent(_device[device_id].buffer_address + event_address, event_buffer);
}

#if defined(USE_DEVICE_LABELS)
// public access to get general info about a controller data. generally accessed from UI
uint8_t * Device::getDeviceName(uint8_t device_id)
{
	if ( device_id > _device_number ) {
		return nullptr;
	}	
	if ( device_id == 0 ) {
		device_id = _selected_device;
	} else {
		device_id--;
	}
	return _device[device_id].name;
}

uint8_t Device::getDinCtrlBehaviour(uint8_t port)
{
	port--;
	return _remote.din_port[port].behaviour;
}

int16_t Device::getDinCtrlIncrement(uint8_t port)
{
	int16_t increment_base;
	port--;
	
	// convert it back
	// calculates the absolute incremental value based on event max and min values:
	//event = getEvent(_remote.din_port[port].event_address);	
					
	//_remote.din_port[port].increment_base = (abs(increment_base) * _adc_max_resolution) / ( (event->data_c << 8 ) | (event->data_d & 0xff) );
	
	//if ( increment_base < 0 ) {
	//	_remote.din_port[port].increment_base *= -1; // invert sign
	//}	
	
	increment_base = 1;
	
	return increment_base;
}
#endif

void Device::configDevice(uint8_t device_id, uint8_t port, uint8_t channel, uint8_t * deviceName)
{
	if ( device_id > _device_number ) {
		return;
	}	
	device_id--;
	port--;
	channel--;
	_device[device_id].active = 1;
	_device[device_id].port = port;
	_device[device_id].chn = channel;

#ifdef USE_DEVICE_LABELS
	if ( deviceName != nullptr ) {
	    //memcpy (_device[device_id].name, deviceName, 16);
	    memset(_device[device_id].name,' ',16);
	    for ( uint8_t i=0; i < 16; i++ ) {
		if ( deviceName[i] == '\0' ) {
		    break;
		}
		_device[device_id].name[i] = deviceName[i];
	    }
	}
#endif
}

void Device::editCtrlsetDevice(int8_t inc, uint8_t port)
{
	CONTROL_DATA event;
	int8_t device;
	
	if ( port == 0 ) {
		port = _edit_ctrl_selected_port;
	} else {
		port--;
	}
	
	if ( _remote.adc_port[port].mode == 0 && inc < 0 ) {
		return;
	}
	
	if ( _remote.adc_port[port].mode == 0 && inc > 0 ) {
		device = 0;
	} else {
		// increment actual device
		device = _remote.adc_port[port].device_id + inc;
	}

	if ( device > _device_number ) {
 		return;
 	}
 	
 	if ( inc > 0 && _device[device].active == 0 ) {
		for ( uint8_t i=device; i < _device_number; i++ ) {
			if ( _device[i].active == 1 ) {
				break;
			}
			device++;
		}
	}
	
	if ( inc < 0 && _device[device].active == 0 ) {
		for ( int8_t i=device; i >= 0; i-- ) {
			if ( _device[i].active == 1 ) {
				break;
			}
			device--;
		}		
	}
	
	if ( device < 0 ) {
		adcMap(port+1, 0, 1); // change based on selected device
	} else {
		adcMap(port+1, device+1, 1); // global controller, doesnt change on select device		
	} 
 
#ifdef USE_DEVICE_LABELS  
	// force update user feedback data
    if ( _controlDataFeedbackCallback != nullptr && _ctrl_mode == 2 ) {
		getDeviceMapEventData(_remote.adc_port[port].device_id, _remote.adc_port[port].event_address, &event);
        //event = getEvent(_remote.adc_port[port].event_address);
        memcpy (_ctrl.device_name, _device[_remote.adc_port[port].device_id].name, 16);
        memcpy (_ctrl.ctrl_name, event.name, 16);
        if ( event.data_label != 65535 ) {
            memcpy (_ctrl.data_label, getDataLabel(_remote.adc_port[port].device_id, event.data_label, _edit_ctrl_last_value), 16);
        } else {
			sprintf ((char*)_ctrl.data_label, "%-8d", event.label_offset + _edit_ctrl_last_value);
            //sprintf (_ctrl.data_label, "%d", event.label_offset + _edit_ctrl_last_value);
        }
        /*
		if ( _data_feedback_show == true ) {
			uCtrl.oled->setDisplayLockState(false);
			_controlDataFeedbackCallback(&_ctrl, _ctrl_mode);
			uCtrl.oled->setDisplayLockState(true);			
		}
		*/ 
    }
#endif
}

void Device::setCtrlMode(uint8_t mode)
{
	noInterrupts();	 	
	_ctrl_mode = mode;
	interrupts();	
}

uint8_t Device::getCtrlMode()
{
	return _ctrl_mode;
}

void Device::editCtrlsetCtrl(int8_t inc, uint8_t port)
{
	CONTROL_DATA event;
	uint8_t device;
	
	if ( port == 0 ) {
		port = _edit_ctrl_selected_port;
	} else {
		port--;
	}

 	_last_buffer_address = _remote.adc_port[port].event_address + inc;

	if ( _last_buffer_address > _device[_remote.adc_port[port].device_id].buffer_size || _last_buffer_address < 0 ) {
 		return;
 	}
 	
 	if ( _remote.adc_port[port].mode == 0 ) {
		device = 0;
	} else {
		device = _remote.adc_port[port].device_id+1;
	}
 	
	adcMap(port+1, device, _last_buffer_address+1);

#ifdef USE_DEVICE_LABELS
	// force update user feedback data
    if ( _controlDataFeedbackCallback != nullptr && _ctrl_mode == 2 ) {
		getDeviceMapEventData(_remote.adc_port[port].device_id, _remote.adc_port[port].event_address, &event);
        //event = getEvent(_remote.adc_port[port].event_address);
        memcpy (_ctrl.device_name, _device[_remote.adc_port[port].device_id].name, 16);
        memcpy (_ctrl.ctrl_name, event.name, 16);
        if ( event.data_label != 65535 ) {
            memcpy (_ctrl.data_label, getDataLabel(_remote.adc_port[port].device_id, event.data_label, _edit_ctrl_last_value), 16);
        } else {
			sprintf ((char*)_ctrl.data_label, "%-8d", event.label_offset + _edit_ctrl_last_value);
            //sprintf (_ctrl.data_label, "%d", event.label_offset + _edit_ctrl_last_value);
        }      
        /*
        if ( _data_feedback_show == true ) {
			uCtrl.oled->setDisplayLockState(false);
			_controlDataFeedbackCallback(&_ctrl, _ctrl_mode);
			uCtrl.oled->setDisplayLockState(true);			
		}
		*/ 
    }
#endif    
}

void Device::adcMap(uint8_t port, uint8_t device_id, uint16_t controllerAddress)
{
	CONTROL_DATA event;
	uint8_t device, mode;
	uint16_t min, max;
	
	if ( device_id == 0 ) {
		// set as selected device
		device = _selected_device;
		mode = 0;
	} else {
		device_id--;
		device = device_id;
		mode = 1;
	}		
	
	if ( controllerAddress > _device[device].buffer_size ) {
		return;
	}
		
	port--;
	controllerAddress--;
	
	// on each adcmap call we need to update min and max to avoid unwanted triggers when change
				
	getDeviceMapEventData(device, controllerAddress, &event);
	//event = getEvent(_remote.adc_port[port].event_address);

	// Get the handle
	switch (event.type) {

		case uctrl::APP_CTRL:	
			min = (event.data_a << 8 ) | (event.data_b & 0xff);
			max = (event.data_c << 8 ) | (event.data_d & 0xff);
			break;

		case uctrl::MIDI_ANALOG:
		case uctrl::MIDI_TRIGGER:
		case uctrl::MIDI_BUTTON:
			// process range
			if ( event.call == uctrl::protocol::midi::Nrpn) {
				min = 0;
				max = (event.data_c << 8 ) | (event.data_d & 0xff);
			} else {	
				min = event.data_c;
				max = event.data_d;
			}
			break;
			
	}
	
	noInterrupts();			
	// set as selected device
	_remote.adc_port[port].device_id = device;
	_remote.adc_port[port].mode = mode;
	_remote.adc_port[port].event_address = controllerAddress;
	_remote.adc_port[port].min = min;
	_remote.adc_port[port].max = max;
	interrupts();	
		
}

void Device::adcMapClear(uint8_t port, uint8_t mode)
{
	if ( port == 0 ) {
		for ( ; port < uCtrl.getAnalogPorts(); port++ ) {
			if ( _remote.adc_port[port].mode == mode ) {
				noInterrupts();							
				_remote.adc_port[port].event_address = 1023;
				_remote.adc_port[port].mode = 0;
				_remote.adc_port[port].min = 0;
				_remote.adc_port[port].max = 127;
				interrupts();
			}
		}
		return;
	}
	
	port--;
	if ( _remote.adc_port[port].mode == mode ) {
		noInterrupts();	
		_remote.adc_port[port].event_address = 1023;
		_remote.adc_port[port].mode = 0;
		_remote.adc_port[port].min = 0;
		_remote.adc_port[port].max = 127;
		interrupts();
	}
}

void Device::dinMapClear(uint8_t port, uint8_t mode)
{
	if ( port == 0 ) {
		for ( ; port < uCtrl.getDigitalPorts(); port++ ) {
			if ( _remote.din_port[port].mode == mode ) {
				noInterrupts();
				_remote.din_port[port].event_address = 1023;
				_remote.din_port[port].mode = 0;
				//_remote.din_port[port].min = 0;
				//_remote.din_port[port].max = 127;
				interrupts();
			}
		}
		return;
	}
	
	port--;
	if ( _remote.din_port[port].mode == mode ) {
		noInterrupts();
		_remote.din_port[port].event_address = 1023;
		_remote.din_port[port].mode = 0;
		//_remote.din_port[port].min = 0;
		//_remote.din_port[port].max = 127;
		interrupts();
	}
}

void Device::dinMap(uint8_t port, uint8_t device_id, uint16_t controllerAddress, uint8_t button_behaviour, int16_t increment_base)
{
	CONTROL_DATA event;
		
	// button_behaviour == 0: Press
	// increment_base is 0 by default
	// for increment_Base to work we need a place to set the last data to base from
	// we can use the adcMap() and dinMap() calls to set the value register on that port.
	// ..., int16_t base_value)
	// _remote.din_port_value[port] = base_value;
	// now on we always calculate the data from this reference (_remote.din_port_value[port] + increment_base)
	
	port--;
		
	noInterrupts();	
	if ( device_id == 0 ) {
		// set as selected device
		_remote.din_port[port].device_id = _selected_device;
		_remote.din_port[port].mode = 0;
	} else {
		device_id--;
		_remote.din_port[port].device_id = device_id;
		_remote.din_port[port].mode = 1;
	}	
	interrupts();

	if ( controllerAddress > _device[device_id].buffer_size ) {
		return;
	}
		
	controllerAddress--;
	
	noInterrupts();
	_remote.din_port[port].event_address = controllerAddress;
#if defined(USE_DEVICE_LABELS)
	_remote.din_port[port].behaviour = button_behaviour; 
#endif	
	interrupts();

	// common usage of button, LOW and HIGH states as users press or not press    
	if ( button_behaviour == 1 ) {
		return;
	}    
        
#if defined(USE_DEVICE_LABELS)		
	// calculates the absolute incremental value based on event max and min values:
	getDeviceMapEventData(_remote.din_port[port].device_id, (uint16_t)_remote.din_port[port].event_address, &event);
	
	noInterrupts();				
	_remote.din_port[port].increment_base = (abs(increment_base) * _adc_max_resolution) / ( (event.data_c << 8 ) | (event.data_d & 0xff) );
	if ( increment_base < 0 ) {
		_remote.din_port[port].increment_base *= -1; // invert sign
	}
	interrupts();
	
	// Get the handle
	switch (event.type) {

		case uctrl::APP_CTRL:
			noInterrupts();
			_remote.din_port[port].min = (event.data_a << 8 ) | (event.data_b & 0xff);
			_remote.din_port[port].max = (event.data_c << 8 ) | (event.data_d & 0xff);
			interrupts();
			break;

		case uctrl::MIDI_ANALOG:
		case uctrl::MIDI_TRIGGER:
		case uctrl::MIDI_BUTTON:
			// process range
			if ( event.call == uctrl::protocol::midi::Nrpn) {
				noInterrupts();
				_remote.din_port[port].min = 0;
				_remote.din_port[port].max = (event.data_c << 8 ) | (event.data_d & 0xff);
				interrupts();
			} else {
				noInterrupts();	
				_remote.din_port[port].min = event.data_c;
				_remote.din_port[port].max = event.data_d;
				interrupts();
			}
			break;
			
	}	
#endif
	
}

void Device::remoteEventHandler(CONTROL_DATA * event, uint8_t device_id, uint16_t value, bool chain_mode, uint8_t interrupted)
{

	switch (event->type) {

		case uctrl::MIDI_ANALOG:
		case uctrl::MIDI_TRIGGER:
		case uctrl::MIDI_BUTTON:
			if ( interrupted == 1 ) {		
				if ( uCtrl.midi != nullptr && _ctrl_mode != 1 ) {
					uCtrl.midi->sendMessage(eventToMidiMsg(event, value, _device[device_id].chn), _device[device_id].port+1, interrupted, event->config);
				}
				return;
			} else {
				break;
			}

		// this is non realtime action, so always return if its interrupted
		case uctrl::APP_CTRL:
			if ( interrupted == 0 ) {	
				if ( _appCtrlDataCallback != nullptr ) {
					_appCtrlDataCallback((event->config << 8 ) | (event->call & 0xff), value);
				}
				if ( _ctrl_mode == 2 ) {
					return;
				}
				break;
			} else {
				return;
			}

	}  

	// the comming piece of code should only be processed if interrupted == 0, or face the consequences... 

#ifdef USE_DEVICE_LABELS
	if ( uCtrl.oled == nullptr )
		return;
	
	// TODO: dont use oled object here... go for callback solution!
    // if event name has a # on it means do not show feedback, do it silent
    if ( event->name[0] != '#' ) {
		// get CONTROL_DATA_INFO buffer data
		if ( _controlDataFeedbackCallback != nullptr ) {
			memcpy (_ctrl.device_name, _device[device_id].name, 16);
			memcpy (_ctrl.ctrl_name, event->name, 16);
			if ( event->data_label != 65535 ) {
				memcpy (_ctrl.data_label, getDataLabel(device_id, event->data_label, value), 16);
			} else {
				sprintf ((char*)_ctrl.data_label, "%-8d", event->label_offset + value);
				//sprintf (_ctrl.data_label, "%d", event->label_offset + value);
			}
			if ( _data_feedback_show == false ) {
				uCtrl.oled->clearDisplay(1,1,1);
				_data_feedback_show = true;
				uCtrl.oled->setDisplayLockState(true);
			}
			_data_feedback_timeout_timer = millis();
			//if ( _data_feedback_show == true ) {
			//	uCtrl.oled->setDisplayLockState(false);
			//	_controlDataFeedbackCallback(&_ctrl, _ctrl_mode);
			//	uCtrl.oled->setDisplayLockState(true);			
			//}
		}
	}
#endif    

	// Handle the next one in the linked list in case it exists and we are told to do so
	//if( (event->next != nullptr) && (chain_mode) )
	//	remoteEventHandler(event->next, value, chain_mode);
}

#ifdef USE_DEVICE_LABELS
uint32_t Device::getDataFeedbackTimeout()
{
    return _data_feedback_timeout_timer;
}

void Device::dataFeedbackCallback()
{
    if ( _controlDataFeedbackCallback != nullptr ) {
        _controlDataFeedbackCallback(&_ctrl, _ctrl_mode);
    }
}

bool Device::showDataFeedback()
{
	return _data_feedback_show;    
}

void Device::setDataFeedback(bool status)
{
    _data_feedback_show = status;
}

#endif

uint16_t Device::getCtrlAdcMin(uint8_t port)
{
    return _remote.adc_port[port].min;
}

uint16_t Device::getCtrlAdcMax(uint8_t port)
{
    return _remote.adc_port[port].max;
}

/*
void Device::sendEvent(uint16_t eventAddress, uint8_t device_id, uint16_t value)
{
	
	CONTROL_DATA * event;

	event = getEvent(eventAddress);
	
	switch ( event->type ) {
		
		// All analogs, same way to proccess it all
		case uctrl::MIDI_ANALOG:
		case uctrl::APP_CTRL:
			// Handler dispatcher
			remoteEventHandler(event, device_id, value, true);
			break;

		case uctrl::MIDI_TRIGGER:

			// TODO: 
			// 1) Handle read cycle time to avoid crosstalk 
			// 2) Handle fast ADC read for drumrolls 
	
			value = (value / 8);        

			// Threshold check
			// PS: the data_b will be used as Threshold data
			if (value > event->data_b) {
			
				// Getting velocity
				// With full range (Too sensitive ?)
				//controllerData = 127 / ((1023 - event->data_b) / (controllerData - event->data_b));
				// Upper range
				value -= 1 ;
			
			}
	
			break;

	}
}
*/
void Device::lockControl(uint8_t remote_port)
{
	if ( uCtrl.ain != nullptr) {
		if ( remote_port == 0 ) {	
			uCtrl.ain->lockAllControls();
		} else {		
			uCtrl.ain->lockControl(remote_port-1);
		}
	}
}
/*
// Special shift button with n taps support
void Device::setShift(uint8_t port, uint8_t behaviour, uint16_t delay, uint8_t polarity)
{

	if ( port == 0 ) {
		// disable shift function
		_shift_port = -1;
	}
	// Real people called artist indexing style
	port--;

	_shift_port = port;
	_shift_delay_max = delay;
	
	_shift_behaviour = behaviour;
	
	if ( polarity )
		_shift_inverted_pol = true;

}
*/

/*
void Device::handleShiftTap(uint8_t value)
{
	
	if( value == HIGH ) {
		if ( _edit_ctrl ) {
			_edit_ctrl = false;
			_subpage = 0;		
			// call external handler in case of external save neeeds
			if ( _onEditCtrlDoneCallback != nullptr ) {
				// TODO: create a _edit_ctrl_changes for global and another _edit_ctrl_changes for device selected
				if ( _edit_ctrl_changes ) {
					_onEditCtrlDoneCallback(0);
				}
				if ( _edit_ctrl_changes ) {
					_onEditCtrlDoneCallback(1);
				}
			}
			_edit_ctrl_changes = false;
			if ( uCtrl.oled != nullptr ) {			
				_data_feedback_show = false;
				uCtrl.oled->setDisplayLockState(false);
				uCtrl.oled->clearDisplay(1,1,1);
			}				
			return;
		}

		if ( (millis() - _shift_timer) <= _shift_delay_min ) {
			return;
		}

		// double press, call the first registred page
		if ( (millis() - _shift_timer) <= _shift_delay_max ) {
			uCtrl.page->setSubPage(0);
			uCtrl.page->setPage(0);
		} else {
			// set page shifted
			uCtrl.page->setSubPage(1);
			if ( uCtrl.oled != nullptr ) {		
				uCtrl.oled->clearDisplay(1, 1, 1);
			}					
		}

		// start time in count
		_shift_timer = millis();

	} else {
		if ( !_edit_ctrl ) {
			// set page not shifted
			uCtrl.page->setSubPage(0);
			if ( uCtrl.oled != nullptr ) {	
				uCtrl.oled->clearDisplay(1, 1, 1);
			}
		}
	}

	return;

}
*/

bool Device::isPressed(uint8_t port)
{

	//...
	port--;
		
/*
#ifdef UMODULAR_ARDUINO_DIN		
	// ARDUINO DIGITAL
	return umodular::driver::module::arduino_digital::getDataRaw(port);
#endif		
*/

	if ( uCtrl.din != nullptr )
    	return uCtrl.din->getDataRaw(port);		
}

void Device::ctrlMap(uint8_t device_id, uint16_t control_id, uint16_t range_min, uint16_t range_max, int16_t label_offset, uint8_t * name)
{	
	_event_data_buffer[0].type   = uctrl::APP_CTRL;
	_event_data_buffer[0].config = (uint8_t) (control_id >> 8);
	_event_data_buffer[0].call   = (uint8_t) control_id; ;
	// Convert 16 bits min resolution value to two 8bits data vars of event
	_event_data_buffer[0].data_a = (uint8_t) (range_min >> 8);    // msb
	_event_data_buffer[0].data_b = (uint8_t) range_min;           // lsb	
	// Convert 16 bits max resolution value to two 8bits data vars of event
	_event_data_buffer[0].data_c = (uint8_t) (range_max >> 8);    // msb
	_event_data_buffer[0].data_d = (uint8_t) range_max;           // lsb	
	
	eventMap(device_id, &_event_data_buffer[0], name, label_offset);
}

//#ifdef USE_MIDI
//
// MIDI SUPPORT
//
uctrl::protocol::midi::MIDI_MESSAGE * Device::eventToMidiMsg(CONTROL_DATA * event, uint16_t value, uint8_t channel)
{
	static uctrl::protocol::midi::MIDI_MESSAGE midi_msg_handle;
	uint8_t sysex_template_id = 0;
	uint8_t counter = 0;
	
	midi_msg_handle.type = (uctrl::protocol::midi::MidiMessageType)event->call; // CC, MIDI_NOTE_ON, SYSEX

	switch (midi_msg_handle.type) {
		
		case uctrl::protocol::midi::Nrpn: 
			// Send NRPN Parameter data
			// Convert two 8bits to one 16bits value
			//event->data_a // msb
			//event->data_b // lsb
			midi_msg_handle.data1 = (event->data_a << 8 ) | (event->data_b & 0xff);
			midi_msg_handle.data2 = value; //rangeMe(value, 0, (event->data_c << 8 ) | (event->data_d & 0xff));
			midi_msg_handle.channel = channel;
			break;

		case uctrl::protocol::midi::Sysex: 	
			
			// sysex indexing data
			sysex_template_id = (uint8_t) event->config;
			
			for ( counter=0; counter < _midi_sysex_template_buffer[sysex_template_id].size; counter++ ) {
				
				switch ( _midi_sysex_template_buffer[sysex_template_id].data[counter].type ) {
					
					case uctrl::SYSEX_TEMPLATE_STATIC:
						midi_msg_handle.sysex[counter] = _midi_sysex_template_buffer[sysex_template_id].data[counter].value;
						break;

					case uctrl::SYSEX_TEMPLATE_DYNAMIC:
					
						switch ( _midi_sysex_template_buffer[sysex_template_id].data[counter].value ) {
							
							case uctrl::SYSEX_TEMPLATE_TRACK_MIDI_CHANNEL:
								midi_msg_handle.sysex[counter] = channel;
								break;

							case uctrl::SYSEX_TEMPLATE_DATA_A:
								midi_msg_handle.sysex[counter] = event->data_a;
								break;
								
							case uctrl::SYSEX_TEMPLATE_DATA_B:
								midi_msg_handle.sysex[counter] = event->data_b;
								break;
								
							case uctrl::SYSEX_TEMPLATE_VALUE:
								midi_msg_handle.sysex[counter] = value; //rangeMe(value, event->data_c, event->data_d);
								break;
																		
						}
						break;
						
					case uctrl::SYSEX_TEMPLATE_ROLAND_CHECKSUM:
						midi_msg_handle.sysex[counter] = rolandChecksum(&midi_msg_handle.sysex[_midi_sysex_template_buffer[sysex_template_id].data[counter].value-1], _midi_sysex_template_buffer[sysex_template_id].size - _midi_sysex_template_buffer[sysex_template_id].data[counter].value);
						break;
																											
				}
				
			}
			
			// data1 will carry sysex template size
			midi_msg_handle.data1 = _midi_sysex_template_buffer[sysex_template_id].size;

			break;		
		
		default:
			midi_msg_handle.data1 = event->data_a; // NOTE if NOTE_ON
			midi_msg_handle.data2 = value; //rangeMe(value, event->data_c, event->data_d); //event->data_b; // Velocity if NOTE_ON
			midi_msg_handle.channel = channel;
			break;

	}

	return &midi_msg_handle;

}

CONTROL_DATA * Device::midiMsgToEvent(uctrl::protocol::midi::MIDI_MESSAGE * msg, uint16_t min_value_range, uint16_t max_value_range, CONTROL_DATA * event, uint8_t config)
{	
	
	switch ( msg->type ) {

		case uctrl::protocol::midi::ControlChange:
			event->type                = uctrl::MIDI_ANALOG;
			event->data_a              = msg->data1;
			event->data_c              = min_value_range;
			event->data_d              = max_value_range;
			event->config = config;
			break;

		case uctrl::protocol::midi::Sysex:  
			event->type                = uctrl::MIDI_ANALOG;
			//event->config 						 = (msg->sysex[0] << 4 ) | (msg->channel);
			event->config							 = msg->channel; // sysex template id reference
			//event->config							 = msg->sysex[0]; // sysex template id
			event->data_a              = msg->data1; // user 1 byte address space
			event->data_b              = msg->data2; // user 1 byte address space				
			event->data_c              = min_value_range;
			event->data_d              = max_value_range; 
			break;
			
		case uctrl::protocol::midi::NoteOn:  
			// Send Note on MIDI Message
			break;	

		case uctrl::protocol::midi::ProgramChange:   
			break;	
			
		case uctrl::protocol::midi::Nrpn:
			event->type                = uctrl::MIDI_ANALOG;
			// Convert 16 bits NRPN param value to two 8bits data vars of event
			event->data_a = (uint8_t) (msg->data1 >> 8);        // msb
			event->data_b = (uint8_t) msg->data1;      // lsb
			// Convert 16 bits NRPN max resolution value to two 8bits data vars of event
			event->data_c = (uint8_t) (max_value_range >> 8);    // msb
			event->data_d = (uint8_t) max_value_range;           // lsb
			event->config = config;
			break;

		default:
			// handle default cases?
			break;

	}
	
	event->call = msg->type;
	
	// Done!
	return event;	
	
}

void Device::midiMap(uint8_t device_id, uctrl::protocol::midi::MIDI_MESSAGE msg, uint16_t min, uint16_t max, int16_t label_offset, uint8_t * eventName, uint8_t config)
{
	eventMap(device_id, midiMsgToEvent(&msg, min, max, &_event_data_buffer[0], config), eventName, label_offset);
}

void Device::sysexTemplateRegister(MIDI_SYSEX_TEMPLATE * sysex_template, uint8_t id)
{
    _midi_sysex_template_buffer[id].size = sysex_template->size;
    for (uint8_t i=0; i < _midi_sysex_template_buffer[id].size; i++) {
        _midi_sysex_template_buffer[id].data[i].type = sysex_template->data[i].type;
        _midi_sysex_template_buffer[id].data[i].value = sysex_template->data[i].value;
    }
    return;
}

uint8_t Device::rolandChecksum(uint8_t * sysex_buffer, uint8_t buffer_size) {

    uint8_t i, checksum;
    
    checksum = 0;
	
    for (i = 0; i < buffer_size; i++) {
		checksum += sysex_buffer[i];
    }	
    
    checksum = checksum % 128;
    checksum = 128 - checksum;
    
    if (checksum == 128) {
		checksum = 0;
	}

    return checksum;

}
//#endif

#ifdef USE_DEVICE_LABELS
void Device::setDataFeedbackTimeout(uint16_t milliseconds)
{
	
}

void Device::setSharedDataLabel(uint8_t device_id)
{
	if ( device_id > _device_number ) {
		return;
	}	
	device_id--;
		
	_event_data_buffer[0].data_label = _shared_datalabel_address;	
	uCtrl.ram->write((uint8_t*)&_event_data_buffer[0], _device[device_id].buffer_address + _device[device_id].buffer_size-1, _lc1024_buffer_id);
}

void Device::setDataLabel(const uint8_t * label, uint16_t range, uint8_t device_id)
{
	uint16_t memory_address;
	DATA_LABEL data_label_buffer;
	
	if ( device_id > _device_number ) {
		return;
	}	
	device_id--;
	
	if ( _device[device_id].label_buffer_size >= _label_buffer_size ) {
		return; // buffer full!
	}
	
	memset(&data_label_buffer.name,' ',16);
	for ( uint8_t i=0; i < 16; i++ ) {
		if ( label[i] == '\0' ) {
			break;
		}
		data_label_buffer.name[i] = label[i];
	}
	data_label_buffer.range = range;
	data_label_buffer.next = 65535;
	
	memory_address = _device[device_id].label_buffer_address + _device[device_id].label_buffer_size;
	
	uCtrl.ram->write((uint8_t*)&data_label_buffer, memory_address, _lc1024_data_label_buffer_id);

	// global event data buffer(the last registred event data buffer are his guy)
	if ( _event_data_buffer[0].data_label == 65535 ) {
		_event_data_buffer[0].data_label = _device[device_id].label_buffer_size;	
		uCtrl.ram->write((uint8_t*)&_event_data_buffer[0], _device[device_id].buffer_address + _device[device_id].buffer_size-1, _lc1024_buffer_id);
		// keep reference in case of shared dlabel request
		_shared_datalabel_address = _device[device_id].label_buffer_size;
	} else {
		// get last data label into buffer chain and set tail
		uCtrl.ram->read((uint8_t*)&data_label_buffer, memory_address-1, _lc1024_data_label_buffer_id);
		data_label_buffer.next = _device[device_id].label_buffer_size;
		uCtrl.ram->write((uint8_t*)&data_label_buffer, memory_address-1, _lc1024_data_label_buffer_id);
	}
		
	_device[device_id].label_buffer_size++;
}
 
const uint8_t * Device::getDataLabel(uint8_t device_id, uint16_t address, uint16_t value)
{
	static DATA_LABEL data_label_buffer;

	do
	{	
		uCtrl.ram->read((uint8_t*)&data_label_buffer, _device[device_id].label_buffer_address + address, _lc1024_data_label_buffer_id);
		if ( value <= data_label_buffer.range ) {
			break;
		}
		address = data_label_buffer.next;
	} while ( address != 65535 );			
		
	return data_label_buffer.name;
}
#endif

} }

//uctrl::module::Device device_module;
