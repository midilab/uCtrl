/*!
 *  @file       page.hpp
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

#ifndef __U_CTRL_PAGE_HPP__
#define __U_CTRL_PAGE_HPP__

#include <Arduino.h>

// auto defines, change per dinamicly setup
#define USE_PAGE_COMPONENT


#ifdef USE_PAGE_COMPONENT	

#define COMPONENT_LINE  8
#define COMPONENT_GRID  2

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#define POT_ADC_RESOLUTION 4096
#else
#define POT_ADC_RESOLUTION 1024
#endif

// for change control
#define INCREMENT -1
#define DECREMENT -2
#define INCREMENT_SECONDARY -3
#define DECREMENT_SECONDARY -4

typedef struct
{
        int8_t shift;      
        int8_t up;
        int8_t down;
        int8_t left;
        int8_t right;    
        int8_t function1;
        int8_t function2;
        int8_t decrementer;
        int8_t incrementer;
        int8_t decrementer_secondary;
        int8_t incrementer_secondary;
        int8_t pot;
} NAV_COMPONENT_CTRL; 

typedef enum
{
        UP,
        DOWN,
        LEFT,
        RIGHT,
} NAV_DIRECTIONS; 

struct PageComponent
{
        // 4 bits = 0 - 15
        uint8_t line:4;
        uint8_t col:4;
        uint8_t selected_line:4;
        uint8_t selected_grid:4;
        uint8_t f1_state:4;
        uint8_t f2_state:4;
        uint8_t line_size:4;
        uint8_t grid_size:4;
        // 8bits bool setup mem
        bool selected:1;
        bool no_hook:1;
        bool no_nav:1;
        bool change_full_state:1;
        bool update_selector:1;
        bool update_selector_view:1;
        bool reserved:2;

        uint8_t x = 0;
        uint8_t y = 0;

        const char * f1 = nullptr;
        const char * f2 = nullptr;

        PageComponent() : 
                line(1), 
                col(1), 
                selected_line(1),
                selected_grid(1),
                f1_state(0),
                f2_state(0),
                line_size(1),
                grid_size(1),
                selected(false),
                no_hook(false),
                no_nav(false),
                change_full_state(false),
                update_selector(false),
                update_selector_view(false)
                {

                }

        // main component view
        virtual void view() {}

        // up, down, left, rigth if line or grid > 1
        virtual void nav(uint8_t dir) {}

        // -1, -2, -3, -4, to escalar values from 0 to ...
        // #define INCREMENT -1
        // #define DECREMENT -2
        // #define INCREMENT_SECONDARY -3
        // #define DECREMENT_SECONDARY -4
        // > 0: escalar value
        // parse data should take care of number formating
        virtual void change(int16_t value) {}

        // option for hold/release button state
        virtual void changeRelease(int16_t data) {}

        virtual void function1() {}

        virtual void function2() {}

        // state can be used for different pruporses other than selected, not selected
        void setF1(const char * string, uint8_t state = 0)
        {
                f1 = string;
                f1_state = state;
        }

        void setF2(const char * string, uint8_t state = 0)
        {
                f2 = string;
                f2_state = state;
        }

        int16_t parseData(int16_t value, int16_t min_value, int16_t max_value, int16_t current_value)
        {
                int16_t new_value;

                if (value < 0) {
                        if (value == INCREMENT || value == INCREMENT_SECONDARY) {
                                value = 1;
                        } else {
                                value = -1;
                        }
                        new_value = current_value + value;
                        if (new_value > max_value) {
                                new_value = max_value; 
                        } else if (new_value < min_value) {
                                new_value = min_value;
                        }
                } else {
                        // use current_value if provided for pick value on pot for example
                        // TODO pick by value and a smooth way of noise handling for this guy
                        new_value = map(value, 0, POT_ADC_RESOLUTION, min_value, max_value);
                }

                return new_value;
        }
};
#endif // USE_PAGE_COMPONENT

namespace uctrl { namespace module { 

#define MAX_SHIFT_HOOKERS_SIZE 4
#define MAX_ACTION_HOOKERS_SIZE 4

typedef enum {
        DIGITAL_EVENT,
        ANALOG_EVENT,
} EVENT_TYPE;

#ifdef USE_PAGE_COMPONENT	
typedef struct 
{
        int8_t selector_line = 0;
        int8_t selector_grid = 0;
        PageComponent * selected_component;
} SUB_PAGE_DATA;

typedef struct 
{
        int8_t ctrl_id = -1;
        void (*callback)() = nullptr;
} HOOK_CTRL_DATA;
#endif

typedef struct 
{
	void (*create)() = nullptr;	
	void (*destroy)() = nullptr;
	void (*refresh)(uint8_t) = nullptr;	
	void (*digital_input)(uint8_t, uint16_t, uint8_t) = nullptr;
	void (*analog_input)(uint8_t, uint16_t, uint8_t) = nullptr;
        uint8_t sub_page_size:4; // max 16 subpages
        uint8_t sub_page:4;
        const char * name = nullptr;
#ifdef USE_PAGE_COMPONENT
        void (*callback_f1)() = nullptr;
        void (*callback_f2)() = nullptr;
        const char * f1 = nullptr;
        const char * f2 = nullptr;
        uint8_t f1_state = 0;
        uint8_t f2_state = 0;
        // each subpage needs a reference pointer to wich is the selected component of the first one to be selected
        SUB_PAGE_DATA * sub_page_data = nullptr;
#endif
} PAGE_DATA;

class Page
{
    public:
        Page();
        ~Page(); 
  
        void init(uint8_t pages_size);
        void setPage(int8_t page);	
        void setSubPage(int8_t sub_page);
        //void isPageSet(uint8_t page_number);
        void processView();
        void processEvent(uint8_t port, uint16_t value, uint8_t type);
        void set(const char * page_name, 
                 void (*page_create_callback)(),
                 void (*page_destroy_callback)(),
                 void (*page_refresh_callback)(uint8_t),
                 void (*page_digital_input_callback)(uint8_t, uint16_t, uint8_t) = nullptr,
                 void (*page_analog_input_callback)(uint8_t, uint16_t, uint8_t) = nullptr,
                 uint8_t sub_page_size = 1);
        const char * getPageName(int8_t page_id = -1);  
        uint8_t getPage();  
        uint8_t getPageSize();
        uint8_t getSubPage();
        uint8_t getSubPageSize();
        bool getCallbackCreate();
        
        bool isAnalogInputSet();
        bool isDigitalInputSet();

#ifdef USE_PAGE_COMPONENT
        void component(PageComponent & comp, uint8_t line, uint8_t grid, bool default_selected = false);
        void clearComponentMap();
        bool processComponentEvent(uint8_t port, uint16_t value);
        void setNavComponentCtrl(int8_t shift = -1, int8_t up = -1, int8_t down = -1, int8_t left = -1, int8_t right = -1, int8_t function1 = -1, int8_t function2 = -1, int8_t decrementer = -1, int8_t incrementer = -1, int8_t decrementer_secondary = -1, int8_t incrementer_secondary = -1);
        void selectComponent(NAV_DIRECTIONS dir);
        void selectComponent(PageComponent & comp);
	void setFunctionDrawCallback(void (*callback)(const char *, const char *, uint8_t, uint8_t)) {
		_function_display_callback = callback;
	}
        void (*_function_display_callback)(const char *, const char *, uint8_t, uint8_t) = nullptr;
        void setNavPot(int8_t pot_id);
        bool _use_nav_pot = false;
        uint8_t _nav_ctrl_guard = 0;
        NAV_COMPONENT_CTRL _nav_ctrl_port;

        // functions f1, f2 hook support(it does not overload in case a component already have a function hook setup)
	void setFunctionHook(const char * f1_string, const char * f2_string, void (*f1_callback)(), void (*f2_callback)());
        void clearFunctionHook();
        //void setShiftFunctionHook(const char * f1_string, const char * f2_string, void (*f1_callback)(), void (*f2_callback)());
        //void clearShiftFunctionHook();
        void setShiftCtrlAction(int8_t control_id, void (*callback)());
        void clearShiftCtrlAction(int8_t control_id);
        void setCtrlAction(int8_t control_id, void (*callback)());
        void clearCtrlAction(int8_t control_id);
        bool isShiftPressed();
#endif
  
    private:
        uint8_t _pages_size;
        uint8_t _page; // selected page
        uint8_t _last_page;  
        bool _page_callback_create;
        bool _page_callback_destroy;
        PAGE_DATA * _page_data = nullptr;

#ifdef USE_PAGE_COMPONENT
        HOOK_CTRL_DATA _shift_hooker[MAX_SHIFT_HOOKERS_SIZE];
        HOOK_CTRL_DATA _action_hooker[MAX_ACTION_HOOKERS_SIZE];
        PageComponent * _component_map[COMPONENT_LINE][COMPONENT_GRID];
        int8_t _selector_line = 0;
        int8_t _selector_grid = 0;
        uint8_t _shift = 0;
#endif
};

} }

#endif
