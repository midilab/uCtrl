/*!
 *  @file       oled.hpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      Oled helper module
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

#ifndef __U_CTRL_OLED_HPP__
#define __U_CTRL_OLED_HPP__

// default library version
#define USE_OLED_U8G2

// U8x8 Oled driver library Support
#ifdef USE_OLED_U8G2
#include "U8g2/U8g2lib.h"
#else
#include "U8g2/U8x8lib.h"
#endif

namespace uctrl { namespace module {

#define BLINK_TIME 250

class Oled
{
    public:
        Oled();
        ~Oled();  

#ifdef USE_OLED_U8G2
        void plug(U8G2 * display_ptr);
        void drawBox(uint8_t y, uint8_t x, uint8_t height, uint8_t width, bool do_blink = false);
        void refreshDisplay();
#else
        void plug(U8X8 * display_ptr);
        void inverseFont(bool inverse);
#endif
        void print(const uint8_t * bitmap8, uint8_t line, uint8_t col, bool do_blink = false);
        void print(String string, uint8_t line, uint8_t col, bool do_blink = false);
        void select(const char ** select_data, uint8_t selected_data, uint8_t line, uint8_t col,  uint8_t height, uint8_t width, uint8_t start_idx = 0);
        void clearDisplay(uint8_t line = 0, uint8_t col = 0, uint8_t count = 0);
        void flipDisplay(uint8_t is_enable);
        void setDisplayLockState(bool state);
        void powerSave(bool state);
        void mergeBitmap(uint8_t * bitmap8_a, uint8_t * bitmap8_b, bool do_blink = false);
        void setTimer(uint32_t time);
	bool blink();
	bool _blink = false;
	uint32_t _blink_timer = 0;

#ifdef USE_OLED_U8G2
        U8G2 * display;
#else
        U8X8 * display;
#endif

    protected:

        bool _lock_display = false;
};

} } 

#endif
