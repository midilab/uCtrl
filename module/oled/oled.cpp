/*!
 *  @file       oled.cpp
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

#include "oled.hpp"

namespace uctrl { namespace module {

Oled::Oled()
{

}

Oled::~Oled()
{

}

#ifdef USE_OLED_U8G2
void Oled::plug(U8G2 * display_ptr)
#else
void Oled::plug(U8X8 * display_ptr)
#endif
{
	display = display_ptr;
	display->begin();
#ifdef USE_OLED_U8G2
/*
  Fontname: -Misc-Fixed-Medium-R-Normal--7-70-75-75-C-50-ISO10646-1
  Copyright: Public domain font.  Share and enjoy.
  Glyphs: 95/1848
  BBX Build Mode: 0
  u8g2_font_5x7_tr
*/
	display->setFont(u8g2_font_5x7_tr);
	display->setFontRefHeightExtendedText();
	display->setFontMode(1);
	display->setDrawColor(2);
	display->setFontPosTop();
	display->setFontDirection(0);
	display->setBusClock(400000);
	
	////display->setFont(u8g2_font_pressstart2p_r); 
	//display->setFont(u8g2_font_amstrad_cpc_extended_r);
	//display->setContrast(1); // 0 to 255
	// code a screen saver function
	//display->setPowerSave(1); // turn off display but keeps ram  
	//display->setPowerSave(0); // turn on display again based on ram content
#else
/*
  Fontname: -FreeType-Press Start 2P-Medium-R-Normal--8-80-72-72-P-69-ISO10646-1
  Copyright: (c) 2011 Cody 
  Glyphs: 96/556
  BBX Build Mode: 3
  Ops, not free, find another one!
*/
    //display->setFont(u8x8_font_pressstart2p_r); 
    //display->setFont(u8x8_font_amstrad_cpc_extended_r);
	display->setFont(u8x8_font_5x7_r); 
    //display->setContrast(1); // 0 to 255
    //display->setPowerSave(1); // turn off display but keeps ram  
    //display->setPowerSave(0); // turn on display again based on ram content
#endif
	display->clear();
}

void Oled::setDisplayLockState(bool state)
{
	_lock_display = state;
}

void Oled::print(String string, uint8_t line, uint8_t col, bool do_blink)
{
	if (do_blink== true && blink())
		return;

	if ( _lock_display == true )
		return;
	
	--col; 
	--line;

#ifdef USE_OLED_U8G2
	display->drawUTF8(col*5, line*8, string.c_str());
#else
	display->drawUTF8(col, line, string.c_str());
#endif
}

void Oled::print(const uint8_t * bitmap8, uint8_t line, uint8_t col, bool do_blink)
{
	if (do_blink == true && blink())
		return;

	if ( _lock_display == true )
		return;	

	--col; 
	--line;

#ifdef USE_OLED_U8G2
	display->drawXBM(col*8, line*8, 8, 8, bitmap8);
#else
	display->drawTile(col, line, 1, bitmap8);
#endif
}

#ifdef USE_OLED_U8G2
void Oled::drawBox(uint8_t y, uint8_t x, uint8_t height, uint8_t width, bool do_blink)
{
	if (do_blink == true && blink())
		return;

	display->drawBox(x, y, width, height);
}
#endif

void Oled::select(const char ** select_data, uint8_t selected_data, uint8_t line, uint8_t col,  uint8_t height, uint8_t width, uint8_t start_idx)
{
	// old school menu
	uint8_t select_size = ((size_t*)select_data)[-1]/2;
	uint8_t offset = 0;
	if ( selected_data >= height ) {
		start_idx += selected_data - height;
	}
	for (uint8_t i=0; i < height; i++) {
		offset = start_idx+i;
		if (offset < select_size) {
			if (selected_data == offset) {
#ifdef USE_OLED_U8G2
                display->drawBox(col-1, (line*8)+(i*8)-8, 120, 8);
            } 
            //print(select_data[offset], line+i, col, false, false, width);
			print(select_data[offset], line+i, col);
        } 
#else
				//print(select_data[offset], line+i, col, false, true, width);
				print(select_data[offset], line+i, col);
			} else {
				//print(select_data[offset], line+i, col, false, false, width);
				print(select_data[offset], line+i, col);
			}
		} else {
			//print("                ", line+i, col, false, false, width);
			print("                ", line+i, col);
		}
#endif
	}
	/*
	// menu apple style
    uint8_t select_size = sizeof(select_data)*sizeof(char**);
    int8_t data_idx = selected_data - (height/2) + 1;
	for (uint8_t i=0; i < height; i++) {
		if (data_idx >= 0 && data_idx < select_size) {
			if (selected_data == data_idx) {
				print(select_data[data_idx], line+i, col, false, true, width);
			} else {
				print(select_data[data_idx], line+i, col, false, false, width);
			}
		} else {
			print("                ", line+i, col, false, false, width);
		}
		data_idx++;
	}
	*/
}

void Oled::clearDisplay(uint8_t line, uint8_t col, uint8_t count)
{
#ifdef USE_OLED_U8G2
	display->clearBuffer();
#else
    display->clear();
#endif
}

#ifdef USE_OLED_U8G2
void Oled::refreshDisplay()
{
	display->sendBuffer();
}
#endif

void Oled::flipDisplay(uint8_t is_enable)
{
    display->setFlipMode(is_enable);
}

void Oled::powerSave(bool state)
{
	display->setPowerSave(state);
}

void Oled::mergeBitmap(uint8_t * bitmap8_a, uint8_t * bitmap8_b, bool do_blink)
{
	if (do_blink == true && blink())
		return;
		
	for (uint8_t i=0; i < 8; i++) {
		bitmap8_a[i] |= bitmap8_b[i];
	}
}

void Oled::setTimer(uint32_t time) {
	// timmer dependent UI visual effects
	if ( (time - _blink_timer) > BLINK_TIME ) {
		_blink = !_blink;
		_blink_timer = time;
	}
}

bool Oled::blink()
{
	return _blink;
}

#ifndef USE_OLED_U8G2
void Oled::inverseFont(bool inverse)
{
	if ( inverse ) {
		display->setInverseFont(1);
	} else {
		display->setInverseFont(0);
	}
}
#endif

} }

