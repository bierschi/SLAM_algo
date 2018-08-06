/*!
 * @file Display.cpp
 */

#include "Display.hpp"
#include "alt_types.h"
#include "sys/alt_stdio.h"
#include "system.h"
#include <string.h>
#include <stdlib.h>
#include <cstdio>

#include "altera_avalon_spi.h"
#include "altera_avalon_spi_regs.h"
#include "altera_avalon_pio_regs.h"
#include "io.h"

#include "lcdfont.hpp"

alt_u8 Display::_currentLine = 0;
alt_u16 Display::_bgcolor = ILI9341_BLACK;

void set_tft_dc(bool bit) {
	alt_u8 i = IORD_8DIRECT(GARFIELD_GPIO_BASE,0);
	if(bit == true) {
		i |= 1 << 7;
	}
	else {
		i &= ~(1 << 7);
	}
	IOWR(GARFIELD_GPIO_BASE, 0, i);
}


void spi_write_byte(alt_u8 byte) {
	alt_avalon_spi_command(SPI_0_BASE, 0, 1, &byte, 0, NULL, 0);
}

void delay_ms(alt_u8 ms) {
	for(volatile alt_u32 i = 0; i < 2000000;i++);
}

void reversestr(char s[])
 {
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }

void itochptr(int n, char s[])
{
	int i, sign;

	if ((sign = n) < 0)  /* record sign */
		n = -n;          /* make n positive */
	i = 0;
	do {       /* generate digits in reverse order */
		s[i++] = n % 10 + '0';   /* get next digit */
	} while ((n /= 10) > 0);     /* delete it */
	if (sign < 0)
		s[i++] = '-';
	s[i] = '\0';
	reversestr(s);
}


Display::Display(alt_16 width, alt_16 height) {
	_width = width;
	_height = height;
	_cp437 = false;
	_rotation = 0;
	_currentLine = 0;
	_bgcolor = ILI9341_BLACK;
}

void Display::init(alt_u16 bgcolor) {
	//TO DO: TOGGLE RESET

	writecommand(0xEF);
		writedata(0x03);
		writedata(0x80);
		writedata(0x02);

	writecommand(0xCF);
		writedata(0x00);
		writedata(0XC1);
		writedata(0X30);

	writecommand(0xED);
		writedata(0x64);
		writedata(0x03);
		writedata(0X12);
		writedata(0X81);

	writecommand(0xE8);
		writedata(0x85);
		writedata(0x00);
		writedata(0x78);

	writecommand(0xCB);
		writedata(0x39);
		writedata(0x2C);
		writedata(0x00);
		writedata(0x34);
		writedata(0x02);

	writecommand(0xF7);
		writedata(0x20);

	writecommand(0xEA);
		writedata(0x00);
		writedata(0x00);

	writecommand(ILI9341_PWCTR1);    //Power control
		writedata(0x23);   //VRH[5:0]

	writecommand(ILI9341_PWCTR2);    //Power control
		writedata(0x10);   //SAP[2:0];BT[3:0]

	writecommand(ILI9341_VMCTR1);    //VCM control
		writedata(0x3e);
		writedata(0x28);

	writecommand(ILI9341_VMCTR2);    //VCM control2
		writedata(0x86);

	writecommand(ILI9341_MADCTL);    // Memory Access Control
		writedata(0x48);

	writecommand(ILI9341_PIXFMT);
		writedata(0x55);

	writecommand(ILI9341_FRMCTR1);
		writedata(0x00);
		writedata(0x18);

	writecommand(ILI9341_DFUNCTR);    // Display Function Control
		writedata(0x08);
		writedata(0x82);
		writedata(0x27);

	writecommand(0xF2);    // 3Gamma Function Disable
		writedata(0x00);

	writecommand(ILI9341_GAMMASET);    //Gamma curve selected
		writedata(0x01);

	writecommand(ILI9341_GMCTRP1);    //Set Gamma
		writedata(0x0F);
		writedata(0x31);
		writedata(0x2B);
		writedata(0x0C);
		writedata(0x0E);
		writedata(0x08);
		writedata(0x4E);
		writedata(0xF1);
		writedata(0x37);
		writedata(0x07);
		writedata(0x10);
		writedata(0x03);
		writedata(0x0E);
		writedata(0x09);
		writedata(0x00);

	writecommand(ILI9341_GMCTRN1);    //Set Gamma
		writedata(0x00);
		writedata(0x0E);
		writedata(0x14);
		writedata(0x03);
		writedata(0x11);
		writedata(0x07);
		writedata(0x31);
		writedata(0xC1);
		writedata(0x48);
		writedata(0x08);
		writedata(0x0F);
		writedata(0x0C);
		writedata(0x31);
		writedata(0x36);
		writedata(0x0F);

	writecommand(ILI9341_SLPOUT);    //Exit Sleep


	delay_ms(120);


	writecommand(ILI9341_DISPON);    //Display on

	_bgcolor = bgcolor;

	fillScreen(_bgcolor);
}

void Display::writecommand(alt_u8 c) {
	set_tft_dc(false);

	spi_write_byte(c);
}

void Display::writedata(alt_u8 c) {
	set_tft_dc(true);

	spi_write_byte(c);
}

void Display::drawPixel(alt_16 x, alt_16 y, alt_u16 color) {
	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

	setAddrWindow(x,y,x+1,y+1);


	set_tft_dc(true);

	spi_write_byte(color >> 8);
	spi_write_byte(color);
}

void Display::drawChar(alt_16 x, alt_16 y, unsigned char c, alt_u16 color, alt_u16 bg, alt_u8 size) {
	if((x >= _width)            || // Clip right
			(y >= _height)           || // Clip bottom
			((x + CHARSIZE_WIDTH * size - 1) < 0) || // Clip left
			((y + CHARSIZE_HEIGHT * size - 1) < 0))   // Clip top
		return;

	if(!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

	for(alt_8 i=0; i<CHARSIZE_WIDTH; i++ ) {
		alt_u8 line;
		if(i < (CHARSIZE_WIDTH-1)) line = font[(c*(CHARSIZE_WIDTH-1))+i];
		else      line = 0x0;
		for(alt_8 j=0; j<CHARSIZE_HEIGHT; j++, line >>= 1) {
			if(line & 0x1) {
				if(size == 1) drawPixel(x+i, y+j, color);
				else          fillRect(x+(i*size), y+(j*size), size, size, color);
			} else if(bg != color) {
				if(size == 1) drawPixel(x+i, y+j, bg);
				else          fillRect(x+i*size, y+j*size, size, size, bg);
			}
		}
	}
}

void Display::writeLine(const char* line, alt_u16 color, alt_u8 size) {
	alt_u16 cursor_x = 0;
	static alt_u16 cursor_y = 0;

	if(_currentLine < 99) {
		_currentLine++;
	}
	else if(_currentLine==99) {
		_currentLine = 1;
	}

	if(_height < cursor_y+(CHARSIZE_HEIGHT*size)) {
		cursor_y = 0;
	}

	fillRect(0, cursor_y, _width, (CHARSIZE_HEIGHT*size)-1, _bgcolor);

	char number[3];
	itochptr(_currentLine, number);
	//sprintf(number, "%i", currentLine);


	if(_currentLine > 9) {
		drawChar(0, cursor_y, number[0], color, _bgcolor, size);
		drawChar(1*(CHARSIZE_WIDTH*size), cursor_y, number[1], color, _bgcolor, size);
		drawChar(2*(CHARSIZE_WIDTH*size), cursor_y, ':', color, _bgcolor, size);
		drawChar(3*(CHARSIZE_WIDTH*size), cursor_y, ' ', color, _bgcolor, size);
		cursor_x = 3*(CHARSIZE_WIDTH*size);
	}
	else if(_currentLine < 10) {
		drawChar(0, cursor_y, number[0], color, _bgcolor, size);
		drawChar(1*(CHARSIZE_WIDTH*size), cursor_y, ':', color, _bgcolor, size);
		drawChar(2*(CHARSIZE_WIDTH*size), cursor_y, ' ', color, _bgcolor, size);
		cursor_x = 2*(CHARSIZE_WIDTH*size);
	}


	for(alt_u16 i = 0; i < strlen(line); i++) {
			if((cursor_x + (CHARSIZE_WIDTH*size)) >= _width) { // Heading off edge?
				cursor_y += CHARSIZE_HEIGHT*size;
				return;
			}
			drawChar(cursor_x, cursor_y, line[i], color, _bgcolor, size);
			cursor_x += CHARSIZE_WIDTH*size;
	}
	cursor_y += CHARSIZE_HEIGHT*size;
}

void Display::fillScreen(alt_u16 color) {
	fillRect(0, 0, _width, _height, color);
}

void Display::fillRect(alt_16 x, alt_16 y, alt_16 w, alt_16 h, alt_u16 color) {

	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= _width) || (y >= _height)) return;
	if((x + w - 1) >= _width)  w = _width  - x;
	if((y + h - 1) >= _height) h = _height - y;

	setAddrWindow(x, y, x+w-1, y+h-1);

	alt_u8 hi = color >> 8, lo = color;


	set_tft_dc(true);


	for(y=h; y>0; y--) {
		for(x=w; x>0; x--) {
			spi_write_byte(hi);
			spi_write_byte(lo);
		}
	}
}

void Display::setAddrWindow(alt_u16 x0, alt_u16 y0, alt_u16 x1, alt_u16 y1) {
	writecommand(ILI9341_CASET); // Column addr set
	writedata(x0 >> 8);
	writedata(x0 & 0xFF);     // XSTART
	writedata(x1 >> 8);
	writedata(x1 & 0xFF);     // XEND

	writecommand(ILI9341_PASET); // Row addr set
	writedata(y0>>8);
	writedata(y0);     // YSTART
	writedata(y1>>8);
	writedata(y1);     // YEND

	writecommand(ILI9341_RAMWR); // write to RAM
}

void Display::cp437(bool x=true) {
	_cp437 = x;
}

void Display::setRotation(alt_u8 m) {

  writecommand(ILI9341_MADCTL);
  _rotation = m % 4; // can't be higher than 3
  switch (_rotation) {
   case 0:
     writedata(MADCTL_MX | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
     break;
   case 1:
     writedata(MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  case 2:
    writedata(MADCTL_MY | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
    break;
   case 3:
     writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  }
}
