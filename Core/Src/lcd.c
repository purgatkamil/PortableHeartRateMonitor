#include "lcd.h"
#include "spi.h"

#define ST7735S_SLPOUT			0x11
#define ST7735S_DISPOFF			0x28
#define ST7735S_DISPON			0x29
#define ST7735S_CASET			0x2a
#define ST7735S_RASET			0x2b
#define ST7735S_RAMWR			0x2c
#define ST7735S_MADCTL			0x36
#define ST7735S_COLMOD			0x3a
#define ST7735S_FRMCTR1			0xb1
#define ST7735S_FRMCTR2			0xb2
#define ST7735S_FRMCTR3			0xb3
#define ST7735S_INVCTR			0xb4
#define ST7735S_PWCTR1			0xc0
#define ST7735S_PWCTR2			0xc1
#define ST7735S_PWCTR3			0xc2
#define ST7735S_PWCTR4			0xc3
#define ST7735S_PWCTR5			0xc4
#define ST7735S_VMCTR1			0xc5
#define ST7735S_GAMCTRP1		0xe0
#define ST7735S_GAMCTRN1		0xe1

#define FONT_WIDTH 5
#define FONT_HEIGHT 8

uint8_t Font8_Table[] =
{
	// @0 ' ' (5 pixels wide)
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //

	// @8 '!' (5 pixels wide)
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x00, //
	0x20, //   #
	0x00, //
	0x00, //

	// @16 '"' (5 pixels wide)
	0x50, //  # #
	0x50, //  # #
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //

	// @24 '#' (5 pixels wide)
	0x28, //   # #
	0x50, //  # #
	0xF8, // #####
	0x50, //  # #
	0xF8, // #####
	0x50, //  # #
	0xA0, // # #
	0x00, //

	// @32 '$' (5 pixels wide)
	0x20, //   #
	0x30, //   ##
	0x60, //  ##
	0x30, //   ##
	0x10, //    #
	0x60, //  ##
	0x20, //   #
	0x00, //

	// @40 '%' (5 pixels wide)
	0x20, //   #
	0x20, //   #
	0x18, //    ##
	0x60, //  ##
	0x10, //    #
	0x10, //    #
	0x00, //
	0x00, //

	// @48 '&' (5 pixels wide)
	0x00, //
	0x38, //   ###
	0x20, //   #
	0x60, //  ##
	0x50, //  # #
	0x78, //  ####
	0x00, //
	0x00, //

	// @56 ''' (5 pixels wide)
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //

	// @64 '(' (5 pixels wide)
	0x10, //    #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x10, //    #
	0x00, //

	// @72 ')' (5 pixels wide)
	0x40, //  #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x40, //  #
	0x00, //

	// @80 '*' (5 pixels wide)
	0x20, //   #
	0x70, //  ###
	0x20, //   #
	0x50, //  # #
	0x00, //
	0x00, //
	0x00, //
	0x00, //

	// @88 '+' (5 pixels wide)
	0x00, //
	0x20, //   #
	0x20, //   #
	0xF8, // #####
	0x20, //   #
	0x20, //   #
	0x00, //
	0x00, //

	// @96 ',' (5 pixels wide)
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x10, //    #
	0x20, //   #
	0x20, //   #
	0x00, //

	// @104 '-' (5 pixels wide)
	0x00, //
	0x00, //
	0x00, //
	0x70, //  ###
	0x00, //
	0x00, //
	0x00, //
	0x00, //

	// @112 '.' (5 pixels wide)
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x20, //   #
	0x00, //
	0x00, //

	// @120 '/' (5 pixels wide)
	0x10, //    #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x40, //  #
	0x40, //  #
	0x80, // #
	0x00, //

	// @128 '0' (5 pixels wide)
	0x20, //   #
	0x50, //  # #
	0x50, //  # #
	0x50, //  # #
	0x50, //  # #
	0x20, //   #
	0x00, //
	0x00, //

	// @136 '1' (5 pixels wide)
	0x60, //  ##
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x70, // #####			//oryginalenie 0xF8
	0x00, //
	0x00, //

	// @144 '2' (5 pixels wide)
	0x20, //   #
	0x50, //  # #
	0x20, //   #
	0x20, //   #
	0x40, //  #
	0x70, //  ###
	0x00, //
	0x00, //

	// @152 '3' (5 pixels wide)
	0x20, //   #
	0x50, //  # #
	0x10, //    #
	0x20, //   #
	0x10, //    #
	0x60, //  ##
	0x00, //
	0x00, //

	// @160 '4' (5 pixels wide)
	0x10, //    #
	0x30, //   ##
	0x50, //  # #
	0x78, //  ####
	0x10, //    #
	0x38, //   ###
	0x00, //
	0x00, //

	// @168 '5' (5 pixels wide)
	0x70, //  ###
	0x40, //  #
	0x60, //  ##
	0x10, //    #
	0x50, //  # #
	0x20, //   #
	0x00, //
	0x00, //

	// @176 '6' (5 pixels wide)
	0x30, //   ##
	0x40, //  #
	0x60, //  ##
	0x50, //  # #
	0x50, //  # #
	0x60, //  ##
	0x00, //
	0x00, //

	// @184 '7' (5 pixels wide)
	0x70, //  ###
	0x50, //  # #
	0x10, //    #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x00, //
	0x00, //

	// @192 '8' (5 pixels wide)
	0x20, //   #
	0x50, //  # #
	0x20, //   #
	0x50, //  # #
	0x50, //  # #
	0x20, //   #
	0x00, //
	0x00, //

	// @200 '9' (5 pixels wide)
	0x30, //   ##
	0x50, //  # #
	0x50, //  # #
	0x30, //   ##
	0x10, //    #
	0x60, //  ##
	0x00, //
	0x00, //

	// @208 ':' (5 pixels wide)
	0x00, //
	0x00, //
	0x20, //   #
	0x00, //
	0x00, //
	0x20, //   #
	0x00, //
	0x00, //

	// @216 ';' (5 pixels wide)
	0x00, //
	0x00, //
	0x10, //    #
	0x00, //
	0x10, //    #
	0x20, //   #
	0x00, //
	0x00, //

	// @224 '<' (5 pixels wide)
	0x00, //
	0x10, //    #
	0x20, //   #
	0xC0, // ##
	0x20, //   #
	0x10, //    #
	0x00, //
	0x00, //

	// @232 '=' (5 pixels wide)
	0x00, //
	0x70, //  ###
	0x00, //
	0x70, //  ###
	0x00, //
	0x00, //
	0x00, //
	0x00, //

	// @240 '>' (5 pixels wide)
	0x00, //
	0x40, //  #
	0x20, //   #
	0x18, //    ##
	0x20, //   #
	0x40, //  #
	0x00, //
	0x00, //

	// @248 '?' (5 pixels wide)
	0x20, //   #
	0x50, //  # #
	0x10, //    #
	0x20, //   #
	0x00, //
	0x20, //   #
	0x00, //
	0x00, //

	// @256 '@' (5 pixels wide)
	0x30, //   ##
	0x48, //  #  #
	0x48, //  #  #
	0x58, //  # ##
	0x48, //  #  #
	0x40, //  #
	0x38, //   ###
	0x00, //

	// @264 'A' (5 pixels wide)
	0x60, //  ##
	0x20, //   #
	0x50, //  # #
	0x70, //  ###
	0x88, // #   #
	0xD8, // ## ##
	0x00, //
	0x00, //

	// @272 'B' (5 pixels wide)
	0xF0, // ####
	0x48, //  #  #
	0x70, //  ###
	0x48, //  #  #
	0x48, //  #  #
	0xF0, // ####
	0x00, //
	0x00, //

	// @280 'C' (5 pixels wide)
	0x70, //  ###
	0x50, //  # #
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x30, //   ##
	0x00, //
	0x00, //

	// @288 'D' (5 pixels wide)
	0xF0, // ####
	0x48, //  #  #
	0x48, //  #  #
	0x48, //  #  #
	0x48, //  #  #
	0xF0, // ####
	0x00, //
	0x00, //

	// @296 'E' (5 pixels wide)
	0xF8, // #####
	0x48, //  #  #
	0x60, //  ##
	0x40, //  #
	0x48, //  #  #
	0xF8, // #####
	0x00, //
	0x00, //

	// @304 'F' (5 pixels wide)
	0xF8, // #####
	0x48, //  #  #
	0x60, //  ##
	0x40, //  #
	0x40, //  #
	0xE0, // ###
	0x00, //
	0x00, //

	// @312 'G' (5 pixels wide)
	0x70, //  ###
	0x40, //  #
	0x40, //  #
	0x58, //  # ##
	0x50, //  # #
	0x30, //   ##
	0x00, //
	0x00, //

	// @320 'H' (5 pixels wide)
	0xE8, // ### #
	0x48, //  #  #
	0x78, //  ####
	0x48, //  #  #
	0x48, //  #  #
	0xE8, // ### #
	0x00, //
	0x00, //

	// @328 'I' (5 pixels wide)
	0x70, //  ###
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x70, //  ###
	0x00, //
	0x00, //

	// @336 'J' (5 pixels wide)
	0x38, //   ###
	0x10, //    #
	0x10, //    #
	0x50, //  # #
	0x50, //  # #
	0x20, //   #
	0x00, //
	0x00, //

	// @344 'K' (5 pixels wide)
	0xD8, // ## ##
	0x50, //  # #
	0x60, //  ##
	0x70, //  ###
	0x50, //  # #
	0xD8, // ## ##
	0x00, //
	0x00, //

	// @352 'L' (5 pixels wide)
	0xE0, // ###
	0x40, //  #
	0x40, //  #
	0x40, //  #
	0x48, //  #  #
	0xF8, // #####
	0x00, //
	0x00, //

	// @360 'M' (5 pixels wide)
	0xD8, // ## ##
	0xD8, // ## ##
	0xD8, // ## ##
	0xA8, // # # #
	0x88, // #   #
	0xD8, // ## ##
	0x00, //
	0x00, //

	// @368 'N' (5 pixels wide)
	0xD8, // ## ##
	0x68, //  ## #
	0x68, //  ## #
	0x58, //  # ##
	0x58, //  # ##
	0xE8, // ### #
	0x00, //
	0x00, //

	// @376 'O' (5 pixels wide)
	0x30, //   ##
	0x48, //  #  #
	0x48, //  #  #
	0x48, //  #  #
	0x48, //  #  #
	0x30, //   ##
	0x00, //
	0x00, //

	// @384 'P' (5 pixels wide)
	0xF0, // ####
	0x48, //  #  #
	0x48, //  #  #
	0x70, //  ###
	0x40, //  #
	0xE0, // ###
	0x00, //
	0x00, //

	// @392 'Q' (5 pixels wide)
	0x30, //   ##
	0x48, //  #  #
	0x48, //  #  #
	0x48, //  #  #
	0x48, //  #  #
	0x30, //   ##
	0x18, //    ##
	0x00, //

	// @400 'R' (5 pixels wide)
	0xF0, // ####
	0x48, //  #  #
	0x48, //  #  #
	0x70, //  ###
	0x48, //  #  #
	0xE8, // ### #
	0x00, //
	0x00, //

	// @408 'S' (5 pixels wide)
	0x70, //  ###
	0x50, //  # #
	0x20, //   #
	0x10, //    #
	0x50, //  # #
	0x70, //  ###
	0x00, //
	0x00, //

	// @416 'T' (5 pixels wide)
	0xF8, // #####
	0xA8, // # # #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x70, //  ###
	0x00, //
	0x00, //

	// @424 'U' (5 pixels wide)
	0xD8, // ## ##
	0x48, //  #  #
	0x48, //  #  #
	0x48, //  #  #
	0x48, //  #  #
	0x30, //   ##
	0x00, //
	0x00, //

	// @432 'V' (5 pixels wide)
	0xD8, // ## ##
	0x88, // #   #
	0x48, //  #  #
	0x50, //  # #
	0x50, //  # #
	0x30, //   ##
	0x00, //
	0x00, //

	// @440 'W' (5 pixels wide)
	0xD8, // ## ##
	0x88, // #   #
	0xA8, // # # #
	0xA8, // # # #
	0xA8, // # # #
	0x50, //  # #
	0x00, //
	0x00, //

	// @448 'X' (5 pixels wide)
	0xD8, // ## ##
	0x50, //  # #
	0x20, //   #
	0x20, //   #
	0x50, //  # #
	0xD8, // ## ##
	0x00, //
	0x00, //

	// @456 'Y' (5 pixels wide)
	0xD8, // ## ##
	0x88, // #   #
	0x50, //  # #
	0x20, //   #
	0x20, //   #
	0x70, //  ###
	0x00, //
	0x00, //

	// @464 'Z' (5 pixels wide)
	0x78, //  ####
	0x48, //  #  #
	0x10, //    #
	0x20, //   #
	0x48, //  #  #
	0x78, //  ####
	0x00, //
	0x00, //

	// @472 '[' (5 pixels wide)
	0x30, //   ##
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x30, //   ##
	0x00, //

	// @480 '\' (5 pixels wide)
	0x80, // #
	0x40, //  #
	0x40, //  #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x10, //    #
	0x00, //

	// @488 ']' (5 pixels wide)
	0x60, //  ##
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x60, //  ##
	0x00, //

	// @496 '^' (5 pixels wide)
	0x20, //   #
	0x20, //   #
	0x50, //  # #
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //

	// @504 '_' (5 pixels wide)
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0xF8, // #####

	// @512 '`' (5 pixels wide)
	0x20, //   #
	0x10, //    #
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //
	0x00, //

	// @520 'a' (5 pixels wide)
	0x00, //
	0x00, //
	0x30, //   ##
	0x10, //    #
	0x70, //  ###
	0x78, //  ####
	0x00, //
	0x00, //

	// @528 'b' (5 pixels wide)
	0xC0, // ##
	0x40, //  #
	0x70, //  ###
	0x48, //  #  #
	0x48, //  #  #
	0xF0, // ####
	0x00, //
	0x00, //

	// @536 'c' (5 pixels wide)
	0x00, //
	0x00, //
	0x70, //  ###
	0x40, //  #
	0x40, //  #
	0x70, //  ###
	0x00, //
	0x00, //

	// @544 'd' (5 pixels wide)
	0x18, //    ##
	0x08, //     #
	0x38, //   ###
	0x48, //  #  #
	0x48, //  #  #
	0x38, //   ###
	0x00, //
	0x00, //

	// @552 'e' (5 pixels wide)
	0x00, //
	0x00, //
	0x70, //  ###
	0x70, //  ###
	0x40, //  #
	0x30, //   ##
	0x00, //
	0x00, //

	// @560 'f' (5 pixels wide)
	0x10, //    #
	0x20, //   #
	0x70, //  ###
	0x20, //   #
	0x20, //   #
	0x70, //  ###
	0x00, //
	0x00, //

	// @568 'g' (5 pixels wide)
	0x00, //
	0x00, //
	0x38, //   ###
	0x48, //  #  #
	0x48, //  #  #
	0x38, //   ###
	0x08, //     #
	0x30, //   ##

	// @576 'h' (5 pixels wide)
	0xC0, // ##
	0x40, //  #
	0x70, //  ###
	0x48, //  #  #
	0x48, //  #  #
	0xE8, // ### #
	0x00, //
	0x00, //

	// @584 'i' (5 pixels wide)
	0x20, //   #
	0x00, //
	0x60, //  ##
	0x20, //   #
	0x20, //   #
	0x70, //  ###
	0x00, //
	0x00, //

	// @592 'j' (5 pixels wide)
	0x20, //   #
	0x00, //
	0x70, //  ###
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0x10, //    #
	0x70, //  ###

	// @600 'k' (5 pixels wide)
	0xC0, // ##
	0x40, //  #
	0x58, //  # ##
	0x70, //  ###
	0x50, //  # #
	0xD8, // ## ##
	0x00, //
	0x00, //

	// @608 'l' (5 pixels wide)
	0x60, //  ##
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x70, //  ###
	0x00, //
	0x00, //

	// @616 'm' (5 pixels wide)
	0x00, //
	0x00, //
	0xD0, // ## #
	0xA8, // # # #
	0xA8, // # # #
	0xA8, // # # #
	0x00, //
	0x00, //

	// @624 'n' (5 pixels wide)
	0x00, //
	0x00, //
	0xF0, // ####
	0x48, //  #  #
	0x48, //  #  #
	0xC8, // ##  #
	0x00, //
	0x00, //

	// @632 'o' (5 pixels wide)
	0x00, //
	0x00, //
	0x30, //   ##
	0x48, //  #  #
	0x48, //  #  #
	0x30, //   ##
	0x00, //
	0x00, //

	// @640 'p' (5 pixels wide)
	0x00, //
	0x00, //
	0xF0, // ####
	0x48, //  #  #
	0x48, //  #  #
	0x70, //  ###
	0x40, //  #
	0xE0, // ###

	// @648 'q' (5 pixels wide)
	0x00, //
	0x00, //
	0x38, //   ###
	0x48, //  #  #
	0x48, //  #  #
	0x38, //   ###
	0x08, //     #
	0x18, //    ##

	// @656 'r' (5 pixels wide)
	0x00, //
	0x00, //
	0x78, //  ####
	0x20, //   #
	0x20, //   #
	0x70, //  ###
	0x00, //
	0x00, //

	// @664 's' (5 pixels wide)
	0x00, //
	0x00, //
	0x30, //   ##
	0x20, //   #
	0x10, //    #
	0x60, //  ##
	0x00, //
	0x00, //

	// @672 't' (5 pixels wide)
	0x00, //
	0x40, //  #
	0xF0, // ####
	0x40, //  #
	0x48, //  #  #
	0x30, //   ##
	0x00, //
	0x00, //

	// @680 'u' (5 pixels wide)
	0x00, //
	0x00, //
	0xD8, // ## ##
	0x48, //  #  #
	0x48, //  #  #
	0x38, //   ###
	0x00, //
	0x00, //

	// @688 'v' (5 pixels wide)
	0x00, //
	0x00, //
	0xC8, // ##  #
	0x48, //  #  #
	0x30, //   ##
	0x30, //   ##
	0x00, //
	0x00, //

	// @696 'w' (5 pixels wide)
	0x00, //
	0x00, //
	0xD8, // ## ##
	0xA8, // # # #
	0xA8, // # # #
	0x50, //  # #
	0x00, //
	0x00, //

	// @704 'x' (5 pixels wide)
	0x00, //
	0x00, //
	0x48, //  #  #
	0x30, //   ##
	0x30, //   ##
	0x48, //  #  #
	0x00, //
	0x00, //

	// @712 'y' (5 pixels wide)
	0x00, //
	0x00, //
	0xD8, // ## ##
	0x50, //  # #
	0x50, //  # #
	0x20, //   #
	0x20, //   #
	0x60, //  ##

	// @720 'z' (5 pixels wide)
	0x00, //
	0x00, //
	0x78, //  ####
	0x50, //  # #
	0x28, //   # #
	0x78, //  ####
	0x00, //
	0x00, //

	// @728 '{' (5 pixels wide)
	0x10, //    #
	0x20, //   #
	0x20, //   #
	0x60, //  ##
	0x20, //   #
	0x20, //   #
	0x10, //    #
	0x00, //

	// @736 '|' (5 pixels wide)
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x20, //   #
	0x00, //

	// @744 '}' (5 pixels wide)
	0x40, //  #
	0x20, //   #
	0x20, //   #
	0x30, //   ##
	0x20, //   #
	0x20, //   #
	0x40, //  #
	0x00, //

	// @752 '~' (5 pixels wide)
	0x00, //
	0x00, //
	0x00, //
	0x28, //   # #
	0x50, //  # #
	0x00, //
	0x00, //
	0x00, //
};

static void lcd_cmd(uint8_t cmd)
{
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void lcd_data(uint8_t data)
{
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

#define CMD(x)			((x) | 0x100)

static void lcd_send(uint16_t value)
{
	if (value & 0x100) {
		lcd_cmd(value);
	} else {
		lcd_data(value);
	}
}

static const uint16_t init_table[] = {
  CMD(ST7735S_FRMCTR1), 0x01, 0x2c, 0x2d,
  CMD(ST7735S_FRMCTR2), 0x01, 0x2c, 0x2d,
  CMD(ST7735S_FRMCTR3), 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d,
  CMD(ST7735S_INVCTR), 0x07,
  CMD(ST7735S_PWCTR1), 0xa2, 0x02, 0x84,
  CMD(ST7735S_PWCTR2), 0xc5,
  CMD(ST7735S_PWCTR3), 0x0a, 0x00,
  CMD(ST7735S_PWCTR4), 0x8a, 0x2a,
  CMD(ST7735S_PWCTR5), 0x8a, 0xee,
  CMD(ST7735S_VMCTR1), 0x0e,
  CMD(ST7735S_GAMCTRP1), 0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22,
                         0x1f, 0x1b, 0x23, 0x37, 0x00, 0x07, 0x02, 0x10,
  CMD(ST7735S_GAMCTRN1), 0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e,
                         0x30, 0x30, 0x39, 0x3f, 0x00, 0x07, 0x03, 0x10,
  CMD(0xf0), 0x01,
  CMD(0xf6), 0x00,
  CMD(ST7735S_COLMOD), 0x05,
  CMD(ST7735S_MADCTL), 0x60, //rotacja o 180 stopni  //0xa0,
};

void lcd_init(void){

	int i = 0;

	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

	for(i = 0; i < sizeof(init_table) / sizeof(uint16_t); i++){
		lcd_send(init_table[i]);
	}

	HAL_Delay(200);

	lcd_cmd(ST7735S_SLPOUT);
	HAL_Delay(120);

	lcd_cmd(ST7735S_DISPON);
}

static void lcd_data16(uint16_t value){

	lcd_data(value >> 8);
	lcd_data(value);
}

#define LCD_OFFSET_X  1
#define LCD_OFFSET_Y  2

static void lcd_set_window(int x, int y, int width, int height)
{
  lcd_cmd(ST7735S_CASET);
  lcd_data16(LCD_OFFSET_X + x);
  lcd_data16(LCD_OFFSET_X + x + width - 1);

  lcd_cmd(ST7735S_RASET);
  lcd_data16(LCD_OFFSET_Y + y);
  lcd_data16(LCD_OFFSET_Y + y + height- 1);
}

static uint16_t frame_buffer[LCD_WIDTH * LCD_HEIGHT];

void lcd_put_pixel(int x, int y, uint16_t color)
{
	if(x >= 0 && x <= LCD_WIDTH && y >= 0 && y <= LCD_HEIGHT){
		frame_buffer[x + y * LCD_WIDTH] = color;
	}
	else if(x < 0){
		frame_buffer[0 + y * LCD_WIDTH] = color;
	}
	else if(x > LCD_WIDTH){
		frame_buffer[LCD_WIDTH + y * LCD_WIDTH] = color;
	}
	else if(y < 0){
		frame_buffer[x + 0 * LCD_WIDTH] = color;
	}
	else if(y > LCD_HEIGHT){
		frame_buffer[x + LCD_HEIGHT * LCD_WIDTH] = color;
	}


}


void fill_with(uint16_t color){

	 int i = LCD_WIDTH * LCD_HEIGHT - 1;
	 while(i >= 0){
		frame_buffer[i] = color;
		--i;
	 }
}

void lcd_copy(void)
{
	lcd_set_window(0, 0, LCD_WIDTH, LCD_HEIGHT);
	lcd_cmd(ST7735S_RAMWR);
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)frame_buffer, sizeof(frame_buffer));
}

void lcd_draw_image(int x, int y, int width, int height, const uint16_t* data)
{
	lcd_set_window(x, y, width, height);

	lcd_cmd(ST7735S_RAMWR);
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)data, width * height * 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void lcd_transfer_done(void)
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

bool lcd_is_busy(void)
{
	if (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY)
		return true;
	else
		return false;
}

void LCD_DisplayChar(uint16_t Xpoint, uint16_t Ypoint, char Acsii_Char, uint16_t Color) {
    const uint8_t Font_Width = FONT_WIDTH; // Szerokość czcionki
    const uint8_t Font_Height = FONT_HEIGHT; // Wysokość czcionki

    uint32_t Char_Offset = (Acsii_Char - ' ') * Font_Height * (Font_Width / 8 + (Font_Width % 8 ? 1 : 0));
    const unsigned char* ptr = &Font8_Table[Char_Offset];

    for (uint16_t Page = 0; Page < Font_Height; Page++) {
        for (uint16_t Column = 0; Column < Font_Width; Column++) {
            if (*ptr & (0x80 >> (Column % 8))) {
                lcd_put_pixel(Xpoint + Column, Ypoint + Page, Color);
            }

            if (Column % 8 == 7) {
                ptr++;
            }
        }

        if (Font_Width % 8 != 0) {
            ptr++;
        }
    }
}

void LCD_DisplayString(uint16_t Xstart, uint16_t Ystart, char* pString, uint16_t Color) {
    const uint8_t Font_Width = FONT_WIDTH; // Szerokość czcionki
    const uint8_t Font_Height = FONT_HEIGHT; // Wysokość czcionki

    while (*pString != '\0') {
        if (Xstart + Font_Width > LCD_WIDTH) {
            Xstart = 0;
            Ystart += Font_Height;
        }

        if (Ystart + Font_Height > LCD_HEIGHT) {
            break; // Wyjście z pętli, jeśli przekroczy wysokość ekranu
        }

        LCD_DisplayChar(Xstart, Ystart, *pString, Color);
        pString++;
        Xstart += Font_Width;
    }
}

/*void lcd_draw_line(int x1, int y1, int x2, int y2, uint16_t color) {
    int dx, dy, sx, sy, err, e2;

    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    sx = (x1 < x2) ? 1 : -1;
    sy = (y1 < y2) ? 1 : -1;
    err = ((dx > dy) ? dx : -dy) / 2;

    while (1) {
        lcd_put_pixel(x1, y1, color);

        if (x1 == x2 && y1 == y2) break;
        e2 = err;
        if (e2 > -dx) { err -= dy; x1 += sx; }
        if (e2 < dy) { err += dx; y1 += sy; }
    }
}*/

void LCD_DrawLine ( int Xstart, int Ystart,
					int Xend, int Yend,
					uint16_t color){


	int Xpoint = Xstart;
	int Ypoint = Ystart;
	int dx = (int)Xend - (int)Xstart >= 0 ? Xend - Xstart : Xstart - Xend;
	int dy = (int)Yend - (int)Ystart <= 0 ? Yend - Ystart : Ystart - Yend;

	// Increment direction, 1 is positive, -1 is counter;
	int XAddway = Xstart < Xend ? 1 : -1;
	int YAddway = Ystart < Yend ? 1 : -1;

	//Cumulative error
	int Esp = dx + dy;

	for (;;){
		//Painted dotted line, 2 point is really virtual

		lcd_put_pixel(Xpoint, Ypoint, color);

        if (2 * Esp >= dy) {
			if (Xpoint == Xend) break;
            Esp += dy;
			Xpoint += XAddway;
        }
        if (2 * Esp <= dx) {
			if (Ypoint == Yend) break;
            Esp += dx;
			Ypoint += YAddway;
        }
	}
}

void lcd_fill_box(int x1, int y1, int x2, int y2, uint16_t color) {
    // Ensure coordinates are within the screen bounds
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 >= LCD_WIDTH) x2 = LCD_WIDTH - 1;
    if (y2 >= LCD_HEIGHT) y2 = LCD_HEIGHT - 1;

    // Iterate over each pixel in the specified rectangle
    for (int x = x1; x <= x2; x++) {
        for (int y = y1; y <= y2; y++) {
            lcd_put_pixel(x, y, color);
        }
    }
}

void lcd_draw_horizontal_line(int x, uint16_t color){
	for(int i = 1; i <= LCD_WIDTH - 1; i++){
		if(i % 2 != 0)
			lcd_put_pixel(i, LCD_HEIGHT - x, color);
	}
}






