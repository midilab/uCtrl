/*!
 *  @file       page.cpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      Page handler module
 *  @version    1.0.0
 *  @author     Romulo Silva
 *  @date       30/10/22
 *  @license    MIT - (c) 2022 - Romulo Silva - contact@midilab.co
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE. 
 */

#include "page.hpp"
	
namespace uctrl { namespace module {

Page::Page()
{
	// PAGE
	_page = 0;
	_pages_size = 0;
	_last_page = 0;
	_page_callback_create = true;
	_page_callback_destroy = false;	
}

Page::~Page()
{
	// delete subpages first
	for (uint8_t i=0; i < _pages_size; i++) {
		delete[] _page_data[i].sub_page_data;
	}
	delete[] _page_data;
}

void Page::init(uint8_t pages_size)
{
	// only init once!
	if (_pages_size != 0) {
		return;
	}

	// alloc once and forever policy!
	if (_page_data == nullptr) {
		_page_data = new PAGE_DATA[pages_size];
		// init page memory
		for (uint8_t i=0; i < pages_size; i++) {
			_page_data[i].create = nullptr;
			_page_data[i].destroy = nullptr;
			_page_data[i].refresh = nullptr;
			_page_data[i].digital_input = nullptr;
			_page_data[i].analog_input = nullptr;
			_page_data[i].name = nullptr;
			_page_data[i].callback_f1 = nullptr;
			_page_data[i].callback_f2 = nullptr;
			_page_data[i].f1 = nullptr;
			_page_data[i].f2 = nullptr;
			_page_data[i].f1_state = 0;
			_page_data[i].f2_state = 0;
			_page_data[i].sub_page_data = nullptr;
		}
	}

	_pages_size = pages_size;
}

const char * Page::getPageName(int8_t page_id)
{
	if (page_id == -1) {
		page_id = _page; 
	}
	return _page_data[_page].name;
}

void Page::processEvent(uint8_t port, uint16_t value, uint8_t type)
{

	switch(type) {
		// DIGITAL
		case DIGITAL_EVENT:
#ifdef USE_PAGE_COMPONENT
			// is this a nav control event?
			// select component, call event...
			if (processComponentEvent(port, value)) {
				return;
			}
#endif
			if ( _page_data[_page].digital_input != nullptr ) {
				_page_data[_page].digital_input(port, value, _page_data[_page].sub_page);
			}
		break;
		// ANALOG
		case ANALOG_EVENT:
#ifdef USE_PAGE_COMPONENT
			// if we have pot as changer and configuration saying pot changer enabled too
			if (port == _nav_ctrl_port.pot && _use_nav_pot) {
				if (_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component != nullptr) {
					_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component->change(value);
				}
				return;
			}
#endif
			if ( _page_data[_page].analog_input != nullptr ) {	
				_page_data[_page].analog_input(port, value, _page_data[_page].sub_page);
			}
		break;
	}

}

//
// PAGE Handler
//
void Page::processView()
{
	// do we have some page callback registred?
    if (_pages_size > 0)  { 
        // Do we need to call destroy() page?
        if ( _page_data[_last_page].destroy != nullptr && _page_callback_destroy ) {
            _page_data[_last_page].destroy();	
	    	_page_callback_destroy = false;	
        }

        // Do we need to call setup() page?
        if ( _page_data[_page].create != nullptr && _page_callback_create ) {
            _page_data[_page].create();
            _page_callback_create = false;
        }

        // Call refresh() page
        if ( _page_data[_page].refresh != nullptr ) {
            _page_data[_page].refresh(_page_data[_page].sub_page);
        }
    }    
}

#ifdef USE_PAGE_COMPONENT
void Page::setFunctionHook(const char * f1_string, const char * f2_string, void (*f1_callback)(), void (*f2_callback)()) {
	_page_data[_page].f1 = f1_string;
	_page_data[_page].f2 = f2_string;
	_page_data[_page].callback_f1 = f1_callback;
	_page_data[_page].callback_f2 = f2_callback;
}

void Page::clearFunctionHook() {
	_page_data[_page].f1 = nullptr;
	_page_data[_page].f2 = nullptr;
	_page_data[_page].callback_f1 = nullptr;
	_page_data[_page].callback_f2 = nullptr;
}

void Page::setShiftCtrlAction(int8_t control_id, void (*callback)())
{
	for (uint8_t i=0; i < MAX_SHIFT_HOOKERS_SIZE; i++) {
		if (_shift_hooker[i].ctrl_id == -1) {
			_shift_hooker[i].ctrl_id = control_id;
			_shift_hooker[i].callback = callback;
			return;
		}
	}
	// buffer full...
}

void Page::clearShiftCtrlAction(int8_t control_id)
{
	for (uint8_t i=0; i < MAX_SHIFT_HOOKERS_SIZE; i++) {
		if (_shift_hooker[i].ctrl_id == control_id) {
			_shift_hooker[i].ctrl_id = -1;
			_shift_hooker[i].callback = nullptr;
			return;
		}
	}
}

void Page::setCtrlAction(int8_t control_id, void (*callback)())
{
	for (uint8_t i=0; i < MAX_ACTION_HOOKERS_SIZE; i++) {
		if (_action_hooker[i].ctrl_id == -1) {
			_action_hooker[i].ctrl_id = control_id;
			_action_hooker[i].callback = callback;
			return;
		}
	}
	// buffer full...
}

void Page::clearCtrlAction(int8_t control_id)
{
	for (uint8_t i=0; i < MAX_ACTION_HOOKERS_SIZE; i++) {
		if (_action_hooker[i].ctrl_id == control_id) {
			_action_hooker[i].ctrl_id = -1;
			_action_hooker[i].callback = nullptr;
			return;
		}
	}
}

bool Page::isShiftPressed()
{
	return _shift;
}

void Page::clearComponentMap()
{
	// do it for all array!
	//memset(_component_map[0], nullptr, sizeof(*PageComponent)*COMPONENT_GRID);
	for (uint8_t i=0; i < COMPONENT_LINE; i++) {
		for (uint8_t j=0; j < COMPONENT_GRID; j++) {
			_component_map[i][j] = nullptr;
		}
	}
}

void Page::component(PageComponent & comp, uint8_t line, uint8_t grid, bool default_selected)
{
	if (line > COMPONENT_LINE) return;
	if (grid > COMPONENT_GRID) return;

	// set line and col reference for compoent before call view
	comp.line = line;
	// only for 2 grids 25 chars per 128 pixels (5pixel fonts size)
	comp.col = grid == 1 ? 1 : 14;

	// for oled pixeling possitioning
	comp.y = ((line - 1) * 8) + 1;
	comp.x = grid == 1 ? 0 : 64;

	--line;
	--grid;

	// selected default in case no user last selection
	if (_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component == nullptr && default_selected) {
		_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component = &comp;
		comp.selected = true;
		_selector_line = line;
		_selector_grid = grid;
		// update selected_line and selected_grid
		comp.selected_line = (_selector_line+2) - comp.line;
		comp.selected_grid = (_selector_grid+1);
	}

	// TODO: maybe we need to change running order at run() on uCtrl class. get page running firstly than din, ain, touch...
	// each run reset the map, needs to be done inside uCtrl.cpp
	// check component width to put comp reference over
	// ex: stepsequencer, 3 lines, 2 grids... registred at 5, 1... so 5,1 5,2 6,1 6,2 7,1 7,2 will have comp reference
	// work with grid to get more free memory... like 8x2, grid 1 half screen left, grid 2 half screen right
	if (comp.line_size > 1 || comp.grid_size > 1) {
		for (uint8_t i=line; i < line+comp.line_size; i++) {
			if (comp.grid_size > 1) {
				for (uint8_t j=grid; j < grid+comp.grid_size; j++) {
					_component_map[i][j] = &comp;
				}
			} else {
				_component_map[i][grid] = &comp;
			}
		}
	} else {
		_component_map[line][grid] = &comp;
	}

	// reset f1 and f2 before call view
	comp.f1 = nullptr;
	comp.f2 = nullptr;

	// calling component view drawer
	comp.view();
	if (_function_display_callback != nullptr && comp.selected) {
		if (comp.f1 != nullptr && comp.f2 != nullptr) {
			// callback to function butons view
			_function_display_callback(comp.f1, comp.f2, comp.f1_state, comp.f2_state);
		} else if (_page_data[_page].callback_f1 != nullptr) {
			_function_display_callback(_page_data[_page].f1, _page_data[_page].f2, _page_data[_page].f1_state, _page_data[_page].f2_state);
		}
	}
	// any updates from view for line/grid?
	if (comp.update_selector_view) {
		_selector_line = comp.line + comp.selected_line - 2;
		_selector_grid = comp.selected_grid-1;
	}
}

void Page::selectComponent(PageComponent & comp)
{
	PageComponent * selected_component = _page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component;
	if (selected_component != &comp && selected_component != nullptr) {
		selected_component->selected = false;
	}
	selected_component = &comp;
	selected_component->selected = true;
	_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component = selected_component;
	_selector_line = _page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component->line - 1;
	_selector_grid = _page_data[_page].sub_page_data[_page_data[_page].sub_page].selector_grid - 1;
}

void Page::selectComponent(NAV_DIRECTIONS dir)
{
	PageComponent * selected_component = _page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component;

	switch (dir) {
		case UP:
			_selector_line = --_selector_line >= 0 ? _selector_line : COMPONENT_LINE-1;
			break;
		case DOWN:
			_selector_line = ++_selector_line < COMPONENT_LINE ? _selector_line : 0;
			break;
		case LEFT:
			_selector_grid = --_selector_grid >= 0 ? _selector_grid : COMPONENT_GRID-1;
			break;
		case RIGHT:
			_selector_grid = ++_selector_grid < COMPONENT_GRID ? _selector_grid : 0;
			break;
	}

	// guard a max depth recursive entrance here to avoid deadlock and stack/heap collapse
	// keep track of calls here
	++_nav_ctrl_guard;
	if (dir == UP || dir == DOWN) {
		if (_nav_ctrl_guard >= COMPONENT_LINE)
			return;
	}
	if (dir == LEFT || dir == RIGHT) {
		if (_nav_ctrl_guard >= COMPONENT_GRID)
			return;
	}

	if (_component_map[_selector_line][_selector_grid] != nullptr) {
		if (_component_map[_selector_line][_selector_grid]->no_nav) {
			// move selection forward... do not select this one
			selectComponent(dir);
			return;
		}
		if (selected_component != _component_map[_selector_line][_selector_grid]) {
			if (selected_component != nullptr) {
				// we only set it false if change page is not in first nav interaction
				selected_component->selected = false;
			}
			selected_component = _component_map[_selector_line][_selector_grid];
			// its a different component
			// update selected_line and selected_grid
			selected_component->selected_line = (_selector_line+2) - selected_component->line;
			selected_component->selected_grid = (_selector_grid+1);
			selected_component->nav(dir);
			selected_component->selected = true;
			// update buffer pointer reference
			_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component = selected_component;
		} else {
			// same component(multi line/grid component)
			// update selected_line and selected_grid
			selected_component->selected_line = (_selector_line+2) - selected_component->line;
			selected_component->selected_grid = (_selector_grid+1);
			selected_component->nav(dir);
			// any updates from view for line/grid?
			if (selected_component->update_selector) {
				_selector_line = selected_component->line + selected_component->selected_line - 2;
				_selector_grid = selected_component->selected_grid-1;
			}
		}
	} else {
		selectComponent(dir);
	}
}

bool Page::processComponentEvent(uint8_t port, uint16_t value)
{
	PageComponent * selected_component = _page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component;

	if (port == _nav_ctrl_port.shift) {
		_shift = value;
		return true;
	}

	// all other functions will be push button like
	if (value == LOW) {
		if (selected_component->change_full_state == false || _shift == true)
			return false;
		
		if (selected_component->change_full_state == true && (port != _nav_ctrl_port.decrementer && port != _nav_ctrl_port.incrementer && port != _nav_ctrl_port.decrementer_secondary && port != _nav_ctrl_port.incrementer_secondary)) 
			return false;
	}

	// check for shift hookers
	if (_shift && _shift_hooker != nullptr) {
		for (uint8_t i=0; i < MAX_SHIFT_HOOKERS_SIZE; i++) {
			if (_shift_hooker[i].ctrl_id == port) {
				_shift_hooker[i].callback();
				return true;
			}
		}
	}

	// check for action hookers
	if (_action_hooker != nullptr) {
		for (uint8_t i=0; i < MAX_SHIFT_HOOKERS_SIZE; i++) {
			if (_action_hooker[i].ctrl_id == port) {
				_action_hooker[i].callback();
				return true;
			}
		}
	}
	
	if (port == _nav_ctrl_port.up) {
		if (_shift) {
			// change page: previous
			setPage(_page-1);
		} else {
			_nav_ctrl_guard = 0;
			selectComponent(UP);
		}
		return true;
	}

	if (port == _nav_ctrl_port.down) {
		if (_shift) {
			// change page: next
			setPage(_page+1);
		} else {
			_nav_ctrl_guard = 0;
			selectComponent(DOWN);
		}
		return true;
	}

	if (port == _nav_ctrl_port.left) {
		if (_shift) {
			// change subpage: previous
			setSubPage(_page_data[_page].sub_page-1);
		} else {
			_nav_ctrl_guard = 0;
			selectComponent(LEFT);
		}
		return true;
	}

	if (port == _nav_ctrl_port.right) {
		if (_shift) {
			// change subpage: next
			setSubPage(_page_data[_page].sub_page+1);
		} else {
			_nav_ctrl_guard = 0;
			selectComponent(RIGHT);
		}
		return true;
	}

	if (port == _nav_ctrl_port.decrementer) {
		if (value == HIGH) {
			selected_component->change(DECREMENT);
		} else if (value == LOW) {
			selected_component->changeRelease(DECREMENT);
		}
		return true;
	}

	if (port == _nav_ctrl_port.incrementer) {
		if (value == HIGH) {
			selected_component->change(INCREMENT);
		} else if (value == LOW) {
			selected_component->changeRelease(INCREMENT);
		}
		return true;
	}

	if (port == _nav_ctrl_port.decrementer_secondary) {
		if (value == HIGH) {
			selected_component->change(DECREMENT_SECONDARY);
		} else if (value == LOW) {
			selected_component->changeRelease(DECREMENT_SECONDARY);
		}
		return true;
	}

	if (port == _nav_ctrl_port.incrementer_secondary) {
		if (value == HIGH) {
			selected_component->change(INCREMENT_SECONDARY);
		} else if (value == LOW) {
			selected_component->changeRelease(INCREMENT_SECONDARY);
		}
		return true;
	}

	if (port == _nav_ctrl_port.function1) {
		if (_page_data[_page].callback_f1 != nullptr && selected_component->no_hook == false)
			_page_data[_page].callback_f1();
		else
			selected_component->function1();
		return true;
	}

	if (port == _nav_ctrl_port.function2) {
		if (_page_data[_page].callback_f2 != nullptr && selected_component->no_hook == false)
			_page_data[_page].callback_f2();
		else
			selected_component->function2();
		return true;
	}

	return false;
}

void Page::setNavComponentCtrl(int8_t shift, int8_t up, int8_t down, int8_t left, int8_t right, int8_t function1, int8_t function2, int8_t decrementer, int8_t incrementer, int8_t decrementer_secondary, int8_t incrementer_secondary)
{
	_nav_ctrl_port.shift = shift;
	_nav_ctrl_port.up = up;
	_nav_ctrl_port.down = down;
	_nav_ctrl_port.left = left;
	_nav_ctrl_port.right = right;
	_nav_ctrl_port.function1 = function1;
	_nav_ctrl_port.function2 = function2;
	_nav_ctrl_port.decrementer = decrementer;
	_nav_ctrl_port.incrementer = incrementer;
	_nav_ctrl_port.decrementer_secondary = decrementer_secondary;
	_nav_ctrl_port.incrementer_secondary = incrementer_secondary;
}

void Page::setNavPot(int8_t pot_id)
{
	_nav_ctrl_port.pot = pot_id;
	_use_nav_pot = true;
}
#endif

bool Page::getCallbackCreate()
{
    return _page_callback_create;    
}

// Register pages callbacks
void Page::set(const char * page_name, void (*page_create_callback)(), void (*page_destroy_callback)(), void (*page_refresh_callback)(uint8_t), void (*page_digital_input_callback)(uint8_t, uint16_t, uint8_t), void (*page_analog_input_callback)(uint8_t, uint16_t, uint8_t), uint8_t sub_page_size)
{
	if (_pages_size == 0 || _page == _pages_size) {
		return;
	}

	// TODO: check array indexing before access
	_page_data[_page].create = page_create_callback;// page digital input	
	_page_data[_page].destroy = page_destroy_callback;// page digital input	
	_page_data[_page].refresh = page_refresh_callback;// page refresh
	_page_data[_page].digital_input = page_digital_input_callback;// page digital input
	_page_data[_page].analog_input = page_analog_input_callback;// page analog input
	_page_data[_page].sub_page_size = sub_page_size;
	_page_data[_page].sub_page = 0;

	_page_data[_page].name = page_name;

#ifdef USE_PAGE_COMPONENT
	// init selected component memory, alloc once and forever policy!
	_page_data[_page].sub_page_data = new SUB_PAGE_DATA[sub_page_size];
	for (uint8_t i=0; i < sub_page_size; i++) {
		_page_data[_page].sub_page_data[i].selected_component = nullptr;
		_page_data[_page].sub_page_data[i].selector_line = 0;
		_page_data[_page].sub_page_data[i].selector_grid = 0;
	}
#endif

	// never go beyond allocated page size scope for this index
	if (_page < _pages_size-1) {
		_page++;
	}
}

uint8_t Page::getPageSize()
{
    return _pages_size;    
}

uint8_t Page::getSubPageSize()
{
    return _page_data[_page].sub_page_size;    
}

void Page::setPage(int8_t page)
{
	if (page >= _pages_size) {
		page = 0;
	} 
	
	if (page < 0) {
		page = _pages_size -1;
	}
	
	_page_callback_create = true;
	_page_callback_destroy = true;		

	// reference for destroy() call before create() new page	
	_last_page = _page;			
	_page = page;	

#ifdef USE_PAGE_COMPONENT
	// set last page-subpage element as not selected 
	if (_page_data[_last_page].sub_page_data[_page_data[_last_page].sub_page].selected_component != nullptr) {
		_page_data[_last_page].sub_page_data[_page_data[_last_page].sub_page].selected_component->selected = false;
		// keep track of user nav grid
		_page_data[_page].sub_page_data[_page_data[_page].sub_page].selector_line = _selector_line;
		_page_data[_page].sub_page_data[_page_data[_page].sub_page].selector_grid = _selector_grid;
	}
		
	// set new page-subpage element as selected 
	if (_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component != nullptr) {
		_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component->selected = true;
		// update grid selectors
		_selector_line = _page_data[_page].sub_page_data[_page_data[_page].sub_page].selector_line;
		_selector_grid = _page_data[_page].sub_page_data[_page_data[_page].sub_page].selector_grid;
	}
#endif
}

void Page::setSubPage(int8_t sub_page)
{
	if (sub_page >= _page_data[_page].sub_page_size) {
		sub_page = 0;
	} 
	
	if (sub_page < 0) {
		sub_page = _page_data[_page].sub_page_size - 1;
	}
	
#ifdef USE_PAGE_COMPONENT
	// set last page-subpage element as not selected 
	if (_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component != nullptr)  {
		_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component->selected = false;
		// keep track of user nav grid
		_page_data[_page].sub_page_data[_page_data[_page].sub_page].selector_line = _selector_line;
		_page_data[_page].sub_page_data[_page_data[_page].sub_page].selector_grid = _selector_grid;
	}
#endif

	_page_data[_page].sub_page = sub_page;

#ifdef USE_PAGE_COMPONENT
	// set new page-subpage element as selected 
	if (_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component != nullptr) {
		_page_data[_page].sub_page_data[_page_data[_page].sub_page].selected_component->selected = true;
		// update grid selectors
		_selector_line = _page_data[_page].sub_page_data[_page_data[_page].sub_page].selector_line;
		_selector_grid = _page_data[_page].sub_page_data[_page_data[_page].sub_page].selector_grid;
	}
#endif
}

uint8_t Page::getSubPage()
{
    return _page_data[_page].sub_page;    
}

bool Page::isAnalogInputSet()
{
    if ( _page_data[_page].analog_input == nullptr ) {
        return false;
    } else {
        return true;
    }
}

bool Page::isDigitalInputSet()
{
    if ( _page_data[_page].digital_input == nullptr ) {
        return false;
    } else {
        return true;
    }
}

uint8_t Page::getPage()
{
	return _page;
}

} }

//uctrl::module::Page page_module;
