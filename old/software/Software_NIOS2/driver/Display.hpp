/*!
 * @file Display.hpp
 */

#include "alt_types.h"

#ifndef DISPLAY_HPP_
#define DISPLAY_HPP_

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0D
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
/*
#define ILI9341_PWCTR6  0xFC

 */

// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

//Definitions for rotating screen
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

#define CHARSIZE_HEIGHT 8
#define CHARSIZE_WIDTH 6

/*!
 * Function for setting the TFT-DC Pin for writing data or commands
 * @param bit: If set false, DC Bit is set low, for sending a command. If set true, DC Bit is set high, for sending data
 */
void set_tft_dc(bool bit);

/*!
 * Function for writing 1 byte to the SPI TFT Slave
 * Actually abstracting the generated SPI Function
 * @param byte to be sent
 */
void spi_write_byte(alt_u8 byte);

/*!
 * Delay Function. WARNING: This function is not accurate and needs to be updated
 */
void delay_ms(alt_u8 ms);

void reversestr(char s[]);

void itochptr(int n, char s[]);

/*!
 *class Display wich contains all necessary methods for witing to LCD
 */
class Display {
public:
    /*!
     * Delete the default constructor
     */
    Display() = delete;

	/*!
	 * Constructor for Display class
	 * @param width is for instancing the display (use 240 as width for used display)
	 * @param height is for instancing the display (use 240 as width for used display)
	 */
	Display(alt_16 width, alt_16 height);

	/*!
	 * Init Function for initializing the LCD
	 * @param bgcolor: This bgcolor is used for filling the screen after initializing and for clearing lines to override
	 */
	void init(alt_u16 bgcolor);

	/*!
	 *Method writecommand sends a command to the LCD
	 *DC Pin low to send a command
	 *@param c: command byte to be written
	 */
	void writecommand(alt_u8 c);

	/*!
	 * Method writedata sends data to the LCD
	 * DC Pin high to send data
	 * @param c: data byte to be written
	 */
	void writedata(alt_u8 c);

	/*!
	 * Method for enabling/disabling cp437 charset
	 * @param x: enabled if true, disabled if false
	 */
	void cp437(bool x);

	/*!
	 * Method for setting the internal address for a x-y coordinate. It is used by drawPixel()
	 * @param x0: The x0 position of the address window
	 * @param y0: The y0 position of the address window
	 * @param x1: The x1 position of the address window
	 * @param y1: The y1 position of the address window
	 */
	void setAddrWindow(alt_u16 x0, alt_u16 y0, alt_u16 x1, alt_u16 y1);

	/*!
	 * Draw Pixel Function for writing char on LCD
	 * @param x: This is the x position of the pixel that should be written
	 * @param y: This is the y position of the pixel that should be written
	 * @param color: The color of the pixel
	 */
	void drawPixel(alt_16 x, alt_16 y, alt_u16 color);

	/*!
	 * Method for drawing char to LCD
	 * @param x is the x position for the character
	 * @param y is the y position for the character
	 * @c is the character to be printed
	 * @color is the textcolor
	 * @bg is the background color
	 * @size is the textsize
	 */
	void drawChar(alt_16 x, alt_16 y, unsigned char c, alt_u16 color, alt_u16 bg, alt_u8 size);

	/*!
	 * Method for writing a line to the screen. Bevor each line the current number is printed, to check the current line.
	 * After 100 lines the number is to 1 again
	 * @param constline: this is the line to be printed
	 * @param color: The textcolor which should be printed
	 * @param size: Is the textsize (1: normal. 2: double size. 3: triple size)
	 */
	void writeLine(const char* constline, alt_u16 color, alt_u8 size);

	/*!
	 * Method fill Rect creates a filled rectangle with one color.
	 * This funtcion is used by fillScreen
	 * @param x: This is the x position where the rectangle should start
	 * @param y: This is the y position where the rectangle should start
	 * @param w: This is the width of the rectangle
	 * @param h: This is the height of the rectangle
	 * @param color: This is the color in which the rectangle should be filled
	 */
	void fillRect(alt_16 x, alt_16 y, alt_16 w, alt_16 h, alt_u16 color);

	/*!
	 * Method for filling Screen with one color. Function uses the fillRect() method
	 * @param color: The color for filling the screen
	 */
	void fillScreen(alt_u16 color);

	/*!
	 * Rotate Screen with parameter m
	 * @param m could take values from 0 to 3 (0: , 1: , 2: , 3: )
	 */
	void setRotation(alt_u8 m);


protected:
	alt_16 _width; /*!<width of the display */
	alt_16 _height; /*!<height of the display */
	alt_u8 _rotation; /*!<rotation of the dsiplay (0-3) */
	bool _cp437; /*!<cp charset enabled or disabled */

private:
	static alt_u8 _currentLine; /*!<current line which gets printed at front of every line */
	static alt_u16 _bgcolor; /*!<used static background color on display */
};



#endif /* DISPLAY_HPP_ */
