/* 
 * File:   ILI9340C.h
 * Author: Louis Eric DiGioia
 * Adapted from partially completed PIC24 code port
 *
 * LCD CS = RB12
 * LCD DC = RB9
 * 
 */

#ifndef ILI9340_H
#define	ILI9340_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef FCY
#define FCY 32000000UL  // PIC Fosc 32 MHz (for built-in delays)
#endif

#include <xc.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <p24FJ64GA002.h>
#include <libpic30.h>
#include "gfxfont.h"    // ASCII font for use with GFX library (from Arduino)

// ILI9340 COMMAND LIST (from Adafruit ILI9340 Arduino code)
#define ILI9340_NOP         0x00
#define ILI9340_SWRESET     0x01
#define ILI9340_RDDID       0x04
#define ILI9340_RDDST       0x09
#define ILI9340_SLPIN       0x10
#define ILI9340_SLPOUT      0x11
#define ILI9340_PTLON       0x12
#define ILI9340_NORON       0x13
#define ILI9340_RDMODE      0x0A
#define ILI9340_RDMADCTL    0x0B
#define ILI9340_RDPIXFMT    0x0C
#define ILI9340_RDIMGFMT    0x0A
#define ILI9340_RDSELFDIAG  0x0F
#define ILI9340_INVOFF      0x20
#define ILI9340_INVON       0x21
#define ILI9340_GAMMASET    0x26
#define ILI9340_DISPOFF     0x28
#define ILI9340_DISPON      0x29
#define ILI9340_CASET       0x2A
#define ILI9340_PASET       0x2B
#define ILI9340_RAMWR       0x2C
#define ILI9340_RAMRD       0x2E
#define ILI9340_PTLAR       0x30
#define ILI9340_MADCTL      0x36
#define ILI9340_MADCTL_MY   0x80
#define ILI9340_MADCTL_MX   0x40
#define ILI9340_MADCTL_MV   0x20
#define ILI9340_MADCTL_ML   0x10
#define ILI9340_MADCTL_RGB  0x00
#define ILI9340_MADCTL_BGR  0x08
#define ILI9340_MADCTL_MH   0x04
#define ILI9340_PIXFMT      0x3A
#define ILI9340_FRMCTR1     0xB1
#define ILI9340_FRMCTR2     0xB2
#define ILI9340_FRMCTR3     0xB3
#define ILI9340_INVCTR      0xB4
#define ILI9340_DFUNCTR     0xB6
#define ILI9340_PWCTR1      0xC0
#define ILI9340_PWCTR2      0xC1
#define ILI9340_PWCTR3      0xC2
#define ILI9340_PWCTR4      0xC3
#define ILI9340_PWCTR5      0xC4
#define ILI9340_VMCTR1      0xC5
#define ILI9340_VMCTR2      0xC7
#define ILI9340_RDID1       0xDA
#define ILI9340_RDID2       0xDB
#define ILI9340_RDID3       0xDC
#define ILI9340_RDID4       0xDD
#define ILI9340_GMCTRP1     0xE0
#define ILI9340_GMCTRN1     0xE1
    
// Color definitions
#define	ILI9340_BLACK       0x0000
#define	ILI9340_BLUE        0x001F
#define	ILI9340_RED         0xF800
#define	ILI9340_GREEN       0x07E0
#define ILI9340_CYAN        0x07FF
#define ILI9340_MAGENTA     0xF81F
#define ILI9340_YELLOW      0xFFE0  
#define ILI9340_WHITE       0xFFFF
#define ILI9340_Navy        0x000F
#define ILI9340_DarkGreen   0x03E0
#define ILI9340_DarkCyan    0x03EF
#define ILI9340_Maroon      0x7800
#define ILI9340_Purple      0x780F
#define ILI9340_Olive       0x7BE0
#define ILI9340_LightGrey   0xC618
#define ILI9340_DarkGrey    0x7BEF
#define ILI9340_Orange      0xFD20
#define ILI9340_GreenYellow 0xAFE5
#define ILI9340_Pink        0xF81F
#define ILI9340_Brown       0x6061
#define ILI9340_LightBrown  0x9945
// Sprite definition special
#define ILI9340_Transparent 0x0001
#define ILI9340_NextRow     0x0002
#define ILI9340_EndSprite   0x0003
    
// Short versions
#define	BLA 0x0000  // black
#define	BLU 0x001F  // blue
#define	RED 0xF800  // red
#define	GRN 0x07E0  // green
#define CYN 0x07FF  // cyan
#define MAG 0xF81F  // magenta
#define YEL 0xFFE0  // yellow
#define WHI 0xFFFF  // white
#define NAV 0x000F  // navy
#define DGR 0x03E0  // dark green
#define DCY 0x03EF  // dark cyan
#define MRN 0x7800  // dark maroon
#define PPL 0x780F  // purple
#define OLV 0x7BE0  // olive
#define LGY 0xC618  // light grey
#define DGY 0x7BEF  // dark grey
#define ONG 0xFD20  // orange
#define GLW 0xAFE5  // green yellow
#define PNK 0xF81F  // pink
#define TRA 0x0001  // transparent
#define SNR 0x0002  // sprite next row
#define SND 0x0003  // sprite end
#define BRN 0x6061	// brown
#define LBR 0x9945	// light brown
    
// Random array of different colors for misc use
static int16_t colorCycle[18] = {BRN, BLU, RED, GRN, CYN, MAG, YEL, NAV, DGR,
                                DCY, MRN, PPL, OLV, LGY, DGY, ONG, GLW, PNK};

// Defines specific to SPI TX on RB10 and RB12
#define ILI9340_CS_LOW()   LATBbits.LATB12 = 0;    // CS = RB12
#define ILI9340_CS_HIGH()  LATBbits.LATB12 = 1;
#define ILI9340_DC_LOW()   LATBbits.LATB10 = 0;    // DC = RB10
#define ILI9340_DC_HIGH()  LATBbits.LATB10 = 1;

// Taken from original code, swap two bytes
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }

// Variables taken from original Arduino library
int16_t WIDTH;        // Display width (pixels) - never changes
int16_t HEIGHT;       // Display height (pixels) - never changes
int _width;
int _height;
int16_t cursor_x;     // x location to print text
int16_t cursor_y;     // y location to print text
uint16_t textcolor;   // background color for print()
uint16_t textbgcolor; // text color for print()
uint8_t textsize_x;   // x-axis size of text to print()
uint8_t textsize_y;   // y-axis size of text to print()
uint8_t rotation;     // Screen rotation (0 - 3)
bool wrap;            // wrap text bool
bool _cp437;          // use correct CP437 charset (default is off)
GFXfont *gfxFont;     // Pointer to text font

// ARROW / CURSOR / SQUARE CODE SECTION (object/struct and methods)

// For menu screen cursor special use
struct Arrow{
    int16_t curr_x;         ///< X coordinate of arrow
    int16_t curr_y;         ///< Y coordinate of arrow
    int16_t arrow_color;    ///< Arrow color
    int16_t arrow_bg_color; ///< Menu screen background color used with arrow
    int16_t size;           ///< Size of arrow to match text (1, 2, or 3)
    bool exists;            ///< If arrow is supposed to be displayed or not
};

void ArrowDefine(struct Arrow *a, int16_t x, int16_t y, int16_t c, int16_t bg, int16_t s); // instantiate arrow object
void MoveArrow(struct Arrow *a, int16_t x, int16_t y); // move arrow on screen by xy pixels
void DrawSelArrow(int16_t x, int16_t y, uint16_t color, uint16_t size);
void RemoveArrow(struct Arrow *a); // remove arrow. Arrow can be reinitialized afterwards

// create arrow, initiate xy coordinates, color, and and size
void ArrowDefine(struct Arrow *a, int16_t x, int16_t y, int16_t c, int16_t bg, int16_t s){
    if(!a->exists){
        // initiate arrow
        a->curr_x = x;
        a->curr_y = y;
        a->arrow_color = c;
        a->arrow_bg_color = bg;
        a->size = s;
        a->exists = 1;
        // draw arrow
        DrawSelArrow(a->curr_x, a->curr_y, a->arrow_color, a->size);
    }
}

// move arrow on x and y axes by x and y pixels
void MoveArrow(struct Arrow *a, int16_t x, int16_t y){
    if(a->exists){
        // first erase current arrow by drawing over it in bg color
        DrawSelArrow(a->curr_x, a->curr_y, a->arrow_bg_color, a->size);
        // then update arrow location and redraw
        a->curr_x = a->curr_x + x;
        a->curr_y = a->curr_y + y;
        DrawSelArrow(a->curr_x, a->curr_y, a->arrow_color, a->size);
    }
}

// Draw arrow pointer/selector for menus (also used for erasing) (never called by main)
void DrawSelArrow(int16_t x, int16_t y, uint16_t color, uint16_t size){
    if(size == 1){
        drawFastVLine(x+1, y+1, 8, color);
        drawFastVLine(x+2, y+2, 6, color);
        drawFastVLine(x+3, y+3, 4, color);
        drawFastVLine(x+4, y+4, 2, color);
    } else if (size == 2){
        drawFastVLine(x+1, y+1, 16, color);
        drawFastVLine(x+2, y+2, 14, color);
        drawFastVLine(x+3, y+3, 12, color);
        drawFastVLine(x+4, y+4, 10, color);
        drawFastVLine(x+5, y+5, 8, color);
        drawFastVLine(x+6, y+6, 6, color);
        drawFastVLine(x+7, y+7, 4, color);
        drawFastVLine(x+8, y+8, 2, color);
    } else if (size == 3){
        drawFastVLine(x+1, y+1, 32, color);
        drawFastVLine(x+2, y+2, 30, color);
        drawFastVLine(x+3, y+3, 28, color);
        drawFastVLine(x+4, y+4, 26, color);
        drawFastVLine(x+5, y+5, 24, color);
        drawFastVLine(x+6, y+6, 22, color);
        drawFastVLine(x+7, y+7, 20, color);
        drawFastVLine(x+8, y+8, 18, color);
        drawFastVLine(x+9, y+9, 16, color);
        drawFastVLine(x+10, y+10, 14, color);
        drawFastVLine(x+11, y+11, 12, color);
        drawFastVLine(x+12, y+12, 10, color);
        drawFastVLine(x+13, y+13, 8, color);
        drawFastVLine(x+14, y+14, 6, color);
        drawFastVLine(x+15, y+15, 4, color);
        drawFastVLine(x+16, y+16, 2, color);
    } else if(size == 4){ // faster method for size 3 arrow
        drawFastVLine(x+2, y+2, 30, color);
        drawFastVLine(x+4, y+4, 26, color);
        drawFastVLine(x+6, y+6, 22, color);
        drawFastVLine(x+8, y+8, 18, color);
        drawFastVLine(x+10, y+10, 14, color);
        drawFastVLine(x+12, y+12, 10, color);
        drawFastVLine(x+14, y+14, 6, color);
        drawFastVLine(x+16, y+16, 2, color);
    } else if(size == 5){ // small square
        fillRect(x, y, 10, 10, color);
    } else if(size == 6){ // medium square
        fillRect(x, y, 20, 20, color);
    } else if(size == 7){ // large square
        fillRect(x, y, 30, 30, color);
    }
}

void RemoveArrow(struct Arrow *a){
    if(a->exists){
        DrawSelArrow(a->curr_x, a->curr_y, a->arrow_bg_color, a->size);
        a->exists = 0;
    }
}



// Taken from original code
void WriteString(unsigned const char* str);
void Adafruit_GFX(int16_t w, int16_t h); // Constructor
void LCD_send_CMD(unsigned char);
void LCD_send_DATA(unsigned char);
void init_ILI9340(void);

// PIC MCU RELATED
void SPI2_WriteByte(unsigned char byte);

// GRAPHICS & TEXT RELATED (adapted from original library)
void writePixel(int16_t x, int16_t y, uint16_t color);
void drawPixel(int16_t x, int16_t y, uint16_t color);
void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h);
void SPI_WRITE16(uint16_t w);
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
void drawChar2(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y);
void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void fillScreen(uint16_t color);

// CURSOR RELATED
void setCursor(int16_t x, int16_t y); 
void setTextColor(uint16_t c);
void setTextSize(uint16_t s);
void setTextColorBG(uint16_t c, uint16_t bg);
void setTextWrap(bool w);

// PICTURE/GRAPHICS RELATED
void drawFCpad(int16_t x, int16_t y, int16_t size); // draw controller graphic at x y

void drawFCpad(int16_t x, int16_t y, int16_t size){
    // draw Famicom gamepad graphic
    fillRect(x, y, 21*size, 9*size, ILI9340_RED);         // base
    fillRect(x+(1*size), y+(1*size), 5*size, 7*size, ILI9340_YELLOW);   // faceplate
    fillRect(x+(6*size), y+(3*size), 14*size, 5*size, ILI9340_YELLOW);
    fillRect(x+(6*size), y+(4*size), 7*size, 3*size, ILI9340_RED);
    fillRect(x+(3*size), y+(4*size), 1*size, 3*size, ILI9340_BLACK);   // buttons
    fillRect(x+(2*size), y+(5*size), 3*size, 1*size, ILI9340_BLACK);
    fillRect(x+(7*size), y+(5*size), 2*size, 1*size, ILI9340_BLACK);
    fillRect(x+(10*size), y+(5*size), 2*size, 1*size, ILI9340_BLACK);
    fillRect(x+(14*size), y+(5*size), 2*size, 2*size, ILI9340_BLACK);
    fillRect(x+(17*size), y+(5*size), 2*size, 2*size, ILI9340_BLACK);
}

// PIC24 specific SPI2 write code
void SPI2_WriteByte(unsigned char byte){
    // generic function to send 8-bit data over SPI2
    // LCD connected to SPI2
    unsigned char dummy;    // to receive garbage data back in SPI
    ILI9340_CS_LOW();       // pull CS down to begin TX
    SPI2BUF = byte;         // send byte to TX buffer
    while(!(SPI2STAT & 1)); // wait for buffer ready
    dummy = SPI2BUF;        // read in dummy data
    ILI9340_CS_HIGH();      // pull CS up to end TX
}

void LCD_send_CMD(unsigned char LCD_CMD){
    ILI9340_DC_LOW();       // pull DC pin low
    SPI2_WriteByte(LCD_CMD); // send command to LCD
    ILI9340_DC_HIGH();      // pull DC pin high
}

void LCD_send_DATA(unsigned char LCD_data){
    SPI2_WriteByte(LCD_data); // send data to LCD
}

// adapted from original library
void init_ILI9340 (void){
    
    ILI9340_CS_HIGH();
    ILI9340_CS_LOW();
    
    LCD_send_CMD(0x1);      // SWRESET (Software Reset)
    __delay_ms(7);          // 5 ms minimum delay after SWRESET
    
    LCD_send_CMD(0xEF);
        LCD_send_DATA(0x03);
        LCD_send_DATA(0x80);
        LCD_send_DATA(0x02);

    LCD_send_CMD(0xCF);  
        LCD_send_DATA(0x00); 
        LCD_send_DATA(0XC1); 
        LCD_send_DATA(0X30); 

    LCD_send_CMD(0xED);  
        LCD_send_DATA(0x64); 
        LCD_send_DATA(0x03); 
        LCD_send_DATA(0X12); 
        LCD_send_DATA(0X81); 

    LCD_send_CMD(0xE8);  
        LCD_send_DATA(0x85); 
        LCD_send_DATA(0x00); 
        LCD_send_DATA(0x78); 

    LCD_send_CMD(0xCB);  
        LCD_send_DATA(0x39); 
        LCD_send_DATA(0x2C); 
        LCD_send_DATA(0x00); 
        LCD_send_DATA(0x34); 
        LCD_send_DATA(0x02); 

    LCD_send_CMD(0xF7);  
        LCD_send_DATA(0x20); 

    LCD_send_CMD(0xEA);  
        LCD_send_DATA(0x00); 
        LCD_send_DATA(0x00);
        
    LCD_send_CMD(ILI9340_PWCTR1);    // Power control 
        LCD_send_DATA(0x23);         // VRH[5:0] 

    LCD_send_CMD(ILI9340_PWCTR2);    // Power control 
        LCD_send_DATA(0x10);         // SAP[2:0];BT[3:0] 

    LCD_send_CMD(ILI9340_VMCTR1);    // VCM control 
        LCD_send_DATA(0x3e);         //
        LCD_send_DATA(0x28); 

    LCD_send_CMD(ILI9340_VMCTR2);    // VCM control2 
        LCD_send_DATA(0x86);         // --

    LCD_send_CMD(ILI9340_MADCTL);    // Memory Access Control 
        LCD_send_DATA(ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);

    LCD_send_CMD(ILI9340_PIXFMT);    
        LCD_send_DATA(0x55); 

    LCD_send_CMD(ILI9340_FRMCTR1);    
        LCD_send_DATA(0x00);  
        LCD_send_DATA(0x18); 

    LCD_send_CMD(ILI9340_DFUNCTR);    // Display Function Control 
        LCD_send_DATA(0x08); 
        LCD_send_DATA(0x82);
        LCD_send_DATA(0x27);  

    LCD_send_CMD(0xF2);               // 3Gamma Function Disable 
        LCD_send_DATA(0x00); 

    LCD_send_CMD(ILI9340_GAMMASET);   // Gamma curve selected 
        LCD_send_DATA(0x01); 

    LCD_send_CMD(ILI9340_GMCTRP1);    // Set Gamma 
        LCD_send_DATA(0x0F); 
        LCD_send_DATA(0x31); 
        LCD_send_DATA(0x2B); 
        LCD_send_DATA(0x0C); 
        LCD_send_DATA(0x0E); 
        LCD_send_DATA(0x08); 
        LCD_send_DATA(0x4E); 
        LCD_send_DATA(0xF1); 
        LCD_send_DATA(0x37); 
        LCD_send_DATA(0x07); 
        LCD_send_DATA(0x10); 
        LCD_send_DATA(0x03); 
        LCD_send_DATA(0x0E); 
        LCD_send_DATA(0x09); 
        LCD_send_DATA(0x00); 

    LCD_send_CMD(ILI9340_GMCTRN1);    // Set Gamma 
        LCD_send_DATA(0x00); 
        LCD_send_DATA(0x0E); 
        LCD_send_DATA(0x14); 
        LCD_send_DATA(0x03); 
        LCD_send_DATA(0x11); 
        LCD_send_DATA(0x07); 
        LCD_send_DATA(0x31); 
        LCD_send_DATA(0xC1); 
        LCD_send_DATA(0x48); 
        LCD_send_DATA(0x08); 
        LCD_send_DATA(0x0F); 
        LCD_send_DATA(0x0C); 
        LCD_send_DATA(0x31); 
        LCD_send_DATA(0x36); 
        LCD_send_DATA(0x0F); 

    LCD_send_CMD(ILI9340_SLPOUT);    // Exit Sleep 
    __delay_ms(120); 		
    LCD_send_CMD(ILI9340_DISPON);    // Display on 
    
    ILI9340_CS_HIGH();
       
}

// The following code comes from the original Adafruit library
//  adapted from its original Arduino form to C89 for PIC24 usage
/**************************************************************************/
/*!
   @brief    Instantiate a GFX context for graphics!
   @param    w   Display width, in pixels
   @param    h   Display height, in pixels
*/
/**************************************************************************/
void Adafruit_GFX(int16_t w, int16_t h){
  _width = WIDTH = w;           // set width
  _height = HEIGHT = h;         // set height
  rotation = 0;                 // no screen rotation
  cursor_y = cursor_x = 0;      // set cursor to starting point
  textsize_x = textsize_y = 1;  // default text size
  textcolor = textbgcolor = ILI9340_BLACK;  // text color black with transparent background
  wrap = true;                  // wrap text
  _cp437 = false;               // no legacy high ASCII
  gfxFont = NULL;               // default font
}


void writePixel(int16_t x, int16_t y, uint16_t color){
  drawPixel(x, y, color);
}

void drawPixel(int16_t x, int16_t y, uint16_t color){
  // Clip first...
  if ((x >= 0) && (x < _width) && (y >= 0) && (y < _height)) 
  {
    // THEN set up transaction (if needed) and draw...
    //startWrite();
    setAddrWindow(x, y, 1, 1);
    SPI_WRITE16(color);
    //endWrite();
  }
 
}

void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h){
    uint16_t x2 = (x1 + w - 1), y2 = (y1 + h - 1);
    LCD_send_CMD(ILI9340_CASET); // Column address set
    SPI_WRITE16(x1);
    SPI_WRITE16(x2);
    LCD_send_CMD(ILI9340_PASET); // Row address set
    SPI_WRITE16(y1);
    SPI_WRITE16(y2);
    LCD_send_CMD(ILI9340_RAMWR); // Write to RAM
}
    
void SPI_WRITE16(uint16_t w){
    unsigned char hi = (w >> 8);         // get high byte of word
    unsigned char lo = (w & 0b11111111); // get low byte of word
    SPI2_WriteByte(hi);
    SPI2_WriteByte(lo);
}  


/**************************************************************************/
/*!
   @brief    Fill a rectangle completely with one color. Update in subclasses if
   desired!
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
    //startWrite();
    int16_t i;
    for (i = x; i < x + w; i++) {
        drawFastVLine(i, y, h, color);
    }
    //endWrite();
}


/**************************************************************************/
/*!
   @brief    Draw a perfectly vertical line (this is often optimized in a
   subclass!)
    @param    x   Top-most x coordinate
    @param    y   Top-most y coordinate
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color){
  //startWrite();
  writeLine(x, y, x, y + h - 1, color);
  //endWrite();
}


/**************************************************************************/
/*!
   @brief    Write a line.  Bresenham's algorithm (Adapted from The Crazy
                Programmer)
    @param    x0  Start point x coordinate
    @param    y0  Start point y coordinate
    @param    x1  End point x coordinate
    @param    y1  End point y coordinate
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color){

  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  
  if (steep){
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }
  
  if (x0 > x1){
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  for (; x0 <= x1; x0++){
    if (steep)
      writePixel(y0, x0, color);
    else
      writePixel(y0, x0, color);
    
    err -= dy;
    if (err < 0){
      y0 += ystep;
      err += dx;
    }
  }

}


/**************************************************************************/
/*!
   @brief    Write a rectangle completely with one color, overwrite in
   subclasses if startWrite is defined!
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
  // Overwrite in subclasses if desired!
  fillRect(x, y, w, h, color);
}


/**************************************************************************/
/*!
   @brief    Write a perfectly vertical line, overwrite in subclasses if
   startWrite is defined!
    @param    x   Top-most x coordinate
    @param    y   Top-most y coordinate
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color){
  // Overwrite in subclasses if startWrite is defined!
  // Can be just writeLine(x, y, x, y+h-1, color);
  // or writeFillRect(x, y, 1, h, color);
  drawFastVLine(x, y, h, color);
}

// Custom / added method
void fillScreen(uint16_t color){ // fill screen with specified color
    fillRect(0, 0, _width, _height, color);
}


// TEXT- AND CHARACTER-HANDLING FUNCTIONS ----------------------------------

/**********************************************************************/
  /*!
    @brief  Set text cursor location
    @param  x    X coordinate in pixels
    @param  y    Y coordinate in pixels
  */
  /**********************************************************************/
  void setCursor(int16_t x, int16_t y){
    cursor_x = x;
    cursor_y = y;
  }


/**********************************************************************/
  /*!
    @brief   Set text font color with transparant background
    @param   c   16-bit 5-6-5 Color to draw text with
    @note    For 'transparent' background, background and foreground
             are set to same color rather than using a separate flag.
  */
  /**********************************************************************/
  void setTextColor(uint16_t c){ textcolor = textbgcolor = c; }
  
 
  void setTextSize(uint16_t s){ textsize_x = textsize_y = s; }


   /**********************************************************************/
  /*!
    @brief   Set text font color with custom background color
    @param   c   16-bit 5-6-5 Color to draw text with
    @param   bg  16-bit 5-6-5 Color to draw background/fill with
  */
  /**********************************************************************/
  void setTextColorBG(uint16_t c, uint16_t bg){
    textcolor = c;
    textbgcolor = bg;
  }


/**********************************************************************/
  /*!
  @brief  Set whether text that is too long for the screen width should
          automatically wrap around to the next line (else clip right).
  @param  w  true for wrapping, false for clipping
  */
  /**********************************************************************/
  void setTextWrap(bool w){ wrap = w; }


// Draw a character
/**************************************************************************/
/*!
   @brief   Draw a single character
    @param    x   Bottom left corner x coordinate
    @param    y   Bottom left corner y coordinate
    @param    c   The 8-bit font-indexed character (likely ASCII)
    @param    color 16-bit 5-6-5 Color to draw character with
    @param    bg 16-bit 5-6-5 Color to fill background with (if same as color,
   no background)
    @param    size  Font magnification level, 1 is 'original' size
*/
/**************************************************************************/
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size){
  drawChar2(x, y, c, color, bg, size, size);
}


// Draw a character
/**************************************************************************/
/*!
   @brief   Draw a single character
    @param    x   Bottom left corner x coordinate
    @param    y   Bottom left corner y coordinate
    @param    c   The 8-bit font-indexed character (likely ASCII)
    @param    color 16-bit 5-6-5 Color to draw chraracter with
    @param    bg 16-bit 5-6-5 Color to fill background with (if same as color,
   no background)
    @param    size_x  Font magnification level in X-axis, 1 is 'original' size
    @param    size_y  Font magnification level in Y-axis, 1 is 'original' size
*/
/**************************************************************************/
void drawChar2(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y) {

    if (!gfxFont){ // 'Classic' built-in font, user-provided font NULL

        // do not draw clipping characters
        if ((x >= _width) ||              // Clip right
            (y >= _height) ||             // Clip bottom
            ((x + 6 * size_x - 1) < 0) || // Clip left
            ((y + 8 * size_y - 1) < 0))   // Clip top
            return;

        // Handle 'classic' charset behavior
        if (!_cp437 && (c >= 176))
            c++;

        //startWrite();
        uint8_t line;
        int8_t i, j;
        for (i = 0; i < 5; i++){ // Char bitmap = 5 columns     // for each column of pixels
            //uint8_t line = pgm_read_byte(&font[c * 5 + i]);
            line = font[c * 5 + i];

            for (j = 0; j < 8; j++, line >>= 1){                // for each pixel in column
                if (line & 1){                                  // draw pixel of character
                    if (size_x == 1 && size_y == 1)
                        writePixel(x + i, y + j, color);
                    else
                        writeFillRect(x + i * size_x, y + j * size_y, size_x, size_y, color);
                }
                else if (bg != color){                          // draw background color
                    if (size_x == 1 && size_y == 1)
                        writePixel(x + i, y + j, bg);
                    else
                        writeFillRect(x + i * size_x, y + j * size_y, size_x, size_y, bg);
                }
            }
        }
        
        // for last column (contains no char pixels) if using non-opaque bg color
        if (bg != color) { // If opaque, draw vertical line for last column
          if (size_x == 1 && size_y == 1)
            writeFastVLine(x + 5, y, 8, bg);
          else
            writeFillRect(x + 5 * size_x, y, size_x, 8 * size_y, bg);
        }
        //endWrite();

    } else{ // Custom font
        // no code
    }
}




void WriteString(unsigned const char* str){
    while(*str){ // Print characters until end of line
        
        // Newline
        if (*str == 10){
            cursor_x = 0;               // Reset x pos to zero
            cursor_y += textsize_y * 8; // Increment y by 1 line
        }
        
        // Carriage return
        else if(*str == 13){
            cursor_x = 0;               // Reset x pos to zero
        }    
            
        // Other characters
        else{
            
            // if text wrapping enabled and text will run off screen
            if (wrap && ((cursor_x + textsize_x * 6) > _width)){
                cursor_x = 0;               // Reset x pos to zero
                cursor_y += textsize_y * 8; // Move to next line
            }
            
            // Write character to screen
            drawChar2(cursor_x, cursor_y, *str, textcolor, textbgcolor, textsize_x, textsize_y);
            cursor_x += textsize_x * 6; // increment cursor on x axis

        }
        
        str++;                      // increment string pointer
        
    }
}

void EraseString(unsigned const char* str, uint16_t screenColor){
    while(*str){ // Print characters until end of line
        
        // Newline
        if (*str == 10){
            cursor_x = 0;               // Reset x pos to zero
            cursor_y += textsize_y * 8; // Increment y by 1 line
        }
        
        // Carriage return
        else if(*str == 13){
            cursor_x = 0;               // Reset x pos to zero
        }    
            
        // Other characters
        else{
            
            // if text wrapping enabled and text will run off screen
            if (wrap && ((cursor_x + textsize_x * 6) > _width)){
                cursor_x = 0;               // Reset x pos to zero
                cursor_y += textsize_y * 8; // Move to next line
            }
            
            // Write over characters with background color
            drawChar2(cursor_x, cursor_y, *str, screenColor, screenColor, textsize_x, textsize_y);
            cursor_x += textsize_x * 6; // increment cursor on x axis

        }
        
        str++;                      // increment string pointer
        
    }
}


// GAME SRPTIE FUNCTIONS AND STRUCTURES (completely custom)

// Mario sprite standing still
int16_t sprite_Mario_Still[189] = {
TRA, TRA, TRA, RED, RED, RED, RED, RED, SNR,
TRA, TRA, RED, RED, RED, RED, RED, RED, RED, RED, RED, SNR,
TRA, TRA, BRN, BRN, BRN, GLW, GLW, BRN, GLW, SNR,
TRA, BRN, GLW, BRN, GLW, GLW, GLW, BRN, GLW, GLW, GLW, SNR,
TRA, BRN, GLW, BRN, BRN, GLW, GLW, GLW, BRN, GLW, GLW, GLW, SNR,
TRA, BRN, BRN, GLW, GLW, GLW, GLW, BRN, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, GLW, GLW, GLW, GLW, GLW, GLW, GLW, SNR,

TRA, TRA, BRN, BRN, RED, BRN, BRN, BRN, SNR,
TRA, BRN, BRN, BRN, RED, BRN, BRN, RED, BRN, BRN, BRN, SNR,
BRN, BRN, BRN, BRN, RED, RED, RED, RED, BRN, BRN, BRN, BRN, SNR,
GLW, GLW, BRN, RED, GLW, RED, RED, GLW, RED, BRN, GLW, GLW, SNR,
GLW, GLW, GLW, RED, RED, RED, RED, RED, RED, GLW, GLW, GLW, SNR,
GLW, GLW, RED, RED, RED, RED, RED, RED, RED, RED, GLW, GLW, SNR,
TRA, TRA, RED, RED, RED, TRA, TRA, RED, RED, RED, SNR,
TRA, BRN, BRN, BRN, TRA, TRA, TRA, TRA, BRN, BRN, BRN, SNR,
BRN, BRN, BRN, BRN, TRA, TRA, TRA, TRA, BRN, BRN, BRN, BRN, SND
};
// Mario Sprite Walking
int16_t sprite_Mario_Walk1[250] = {
TRA, TRA, TRA, TRA, TRA, RED, RED, RED, RED, RED, SNR,
TRA, TRA, TRA, TRA, RED, RED, RED, RED, RED, RED, RED, RED, RED, SNR,
TRA, TRA, TRA, TRA, BRN, BRN, BRN, GLW, GLW, BRN, GLW, SNR,
TRA, TRA, TRA, BRN, GLW, BRN, GLW, GLW, GLW, BRN, GLW, GLW, GLW, SNR,
TRA, TRA, TRA, BRN, GLW, BRN, BRN, GLW, GLW, GLW, BRN, GLW, GLW, GLW, SNR,
TRA, TRA, TRA, BRN, BRN, GLW, GLW, GLW, GLW, BRN, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, TRA, TRA, GLW, GLW, GLW, GLW, GLW, GLW, GLW, SNR,

TRA, TRA, BRN, BRN, BRN, BRN, RED, RED, BRN, BRN, SNR,
GLW, GLW, BRN, BRN, BRN, BRN, RED, RED, RED, BRN, BRN, BRN, GLW, GLW, GLW, SNR,
GLW, GLW, GLW, TRA, BRN, BRN, RED, GLW, RED, RED, RED, BRN, BRN, GLW, GLW, SNR,
GLW, GLW, TRA, TRA, RED, RED, RED, RED, RED, RED, RED, TRA, TRA, BRN, SNR,
TRA, TRA, TRA, RED, RED, RED, RED, RED, RED, RED, RED, RED, BRN, BRN, SNR,
TRA, TRA, RED, RED, RED, RED, RED, RED, RED, RED, RED, RED, BRN, BRN, SNR,
TRA, BRN, BRN, RED, RED, RED, TRA, TRA, TRA, RED, RED, RED, BRN, BRN, SNR,
TRA, BRN, BRN, BRN, SNR,
TRA, TRA, BRN, BRN, BRN, SND
};
int16_t sprite_Mario_Walk2[250] = {
TRA, TRA, TRA, TRA, TRA, RED, RED, RED, RED, RED, SNR,
TRA, TRA, TRA, TRA, RED, RED, RED, RED, RED, RED, RED, RED, RED, SNR,
TRA, TRA, TRA, TRA, BRN, BRN, BRN, GLW, GLW, BRN, GLW, SNR,
TRA, TRA, TRA, BRN, GLW, BRN, GLW, GLW, GLW, BRN, GLW, GLW, GLW, SNR,
TRA, TRA, TRA, BRN, GLW, BRN, BRN, GLW, GLW, GLW, BRN, GLW, GLW, GLW, SNR,
TRA, TRA, TRA, BRN, BRN, GLW, GLW, GLW, GLW, BRN, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, TRA, TRA, GLW, GLW, GLW, GLW, GLW, GLW, GLW, SNR,

TRA, TRA, TRA, TRA, BRN, BRN, RED, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, BRN, BRN, BRN, BRN, RED, RED, BRN, BRN, SNR,
TRA, TRA, TRA, BRN, BRN, BRN, RED, RED, GLW, RED, RED, GLW, SNR,
TRA, TRA, TRA, BRN, BRN, BRN, BRN, RED, RED, RED, RED, RED, SNR,
TRA, TRA, TRA, RED, BRN, BRN, GLW, GLW, GLW, RED, RED, RED, SNR,
TRA, TRA, TRA, TRA, RED, BRN, GLW, GLW, RED, RED, RED, SNR,
TRA, TRA, TRA, TRA, TRA, RED, RED, RED, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, TRA, TRA, BRN, BRN, BRN, BRN, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, TRA, TRA, BRN, BRN, BRN, BRN, SND
};
int16_t sprite_Mario_Walk3[250] = {
SNR,
TRA, TRA, TRA, TRA, TRA, TRA, RED, RED, RED, RED, RED, SNR,
TRA, TRA, TRA, TRA, TRA, RED, RED, RED, RED, RED, RED, RED, RED, RED, SNR,
TRA, TRA, TRA, TRA, TRA, BRN, BRN, BRN, GLW, GLW, BRN, GLW, SNR,
TRA, TRA, TRA, TRA, BRN, GLW, BRN, GLW, GLW, GLW, BRN, GLW, GLW, GLW, SNR,
TRA, TRA, TRA, TRA, BRN, GLW, BRN, BRN, GLW, GLW, GLW, BRN, GLW, GLW, GLW, SNR,
TRA, TRA, TRA, TRA, BRN, BRN, GLW, GLW, GLW, GLW, BRN, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, TRA, TRA, TRA, GLW, GLW, GLW, GLW, GLW, GLW, GLW, SNR,

TRA, TRA, TRA, TRA, TRA, BRN, BRN, BRN, BRN, RED, BRN, TRA, GLW, SNR,
TRA, TRA, TRA, TRA, GLW, BRN, BRN, BRN, BRN, BRN, BRN, GLW, GLW, GLW, SNR,
TRA, TRA, TRA, GLW, GLW, RED, BRN, BRN, BRN, BRN, BRN, GLW, GLW, SNR,
TRA, TRA, TRA, BRN, BRN, RED, RED, RED, RED, RED, RED, RED, SNR,
TRA, TRA, TRA, BRN, RED, RED, RED, RED, RED, RED, RED, RED, SNR,
TRA, TRA, BRN, BRN, RED, RED, RED, TRA, RED, RED, RED, SNR,
TRA, TRA, BRN, TRA, TRA, TRA, TRA, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, TRA, TRA, TRA, TRA, BRN, BRN, BRN, BRN, SND
};
// Mario sprite jumping
int16_t sprite_Mario_Jump[300] = {
TRA, TRA, TRA, TRA, TRA, TRA, TRA, TRA, TRA, TRA, TRA, TRA, TRA, GLW, GLW, GLW, SNR,
TRA, TRA, TRA, TRA, TRA, TRA, RED, RED, RED, RED, RED, TRA, TRA, GLW, GLW, GLW, SNR,
TRA, TRA, TRA, TRA, TRA, RED, RED, RED, RED, RED, RED, RED, RED, RED, GLW, GLW, SNR,
TRA, TRA, TRA, TRA, TRA, BRN, BRN, BRN, GLW, GLW, BRN, GLW, TRA, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, TRA, BRN, GLW, BRN, GLW, GLW, GLW, BRN, GLW, GLW, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, TRA, BRN, GLW, BRN, BRN, GLW, GLW, GLW, BRN, GLW, GLW, GLW, BRN, SNR,
TRA, TRA, TRA, TRA, BRN, BRN, GLW, GLW, GLW, GLW, BRN, BRN, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, TRA, TRA, TRA, GLW, GLW, GLW, GLW, GLW, GLW, GLW, BRN, SNR,

TRA, TRA, BRN, BRN, BRN, BRN, BRN, RED, BRN, BRN, BRN, RED, BRN, SNR,
TRA, BRN, BRN, BRN, BRN, BRN, BRN, BRN, RED, BRN, BRN, BRN, RED, TRA, TRA, BRN, SNR,
GLW, GLW, BRN, BRN, BRN, BRN, BRN, BRN, RED, RED, RED, RED, RED, TRA, TRA, BRN, SNR,
GLW, GLW, GLW, TRA, RED, RED, BRN, RED, RED, GLW, RED, RED, GLW, RED, BRN, BRN, SNR,
TRA, GLW, TRA, BRN, RED, RED, RED, RED, RED, RED, RED, RED, RED, RED, BRN, BRN, SNR,
TRA, TRA, BRN, BRN, BRN, RED, RED, RED, RED, RED, RED, RED, RED, RED, BRN, BRN, SNR,
TRA, BRN, BRN, BRN, RED, RED, RED, RED, RED, RED, RED, SNR,
TRA, BRN, TRA, TRA, RED, RED, RED, RED, SND
};
// Mario sprite death
int16_t sprite_Mario_Death[300] = {
TRA, TRA, TRA, TRA, TRA, TRA, RED, RED, RED, RED, SNR,
TRA, TRA, TRA, GLW, TRA, RED, RED, RED, RED, RED, RED, TRA, GLW, SNR,
TRA, GLW, GLW, GLW, BRN, GLW, BRN, GLW, GLW, BRN, GLW, BRN, GLW, GLW, GLW, SNR,
TRA, GLW, GLW, BRN, BRN, GLW, BRN, GLW, GLW, BRN, GLW, BRN, BRN, GLW, GLW, SNR,
TRA, GLW, GLW, BRN, BRN, BRN, GLW, GLW, GLW, GLW, BRN, BRN, BRN, GLW, GLW, SNR,
TRA, TRA, TRA, BRN, BRN, BRN, BRN, GLW, GLW, BRN, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, TRA, BRN, GLW, BRN, BRN, BRN, BRN, GLW, BRN, SNR,
TRA, TRA, TRA, TRA, BRN, GLW, GLW, GLW, GLW, GLW, GLW, BRN, SNR,
TRA, TRA, TRA, RED, RED, RED, GLW, GLW, GLW, GLW, RED, RED, RED, SNR,
TRA, TRA, BRN, BRN, RED, RED, BRN, BRN, BRN, BRN, RED, RED, BRN, BRN, SNR,
TRA, TRA, BRN, BRN, BRN, RED, RED, BRN, BRN, RED, RED, BRN, BRN, BRN, SNR,
TRA, TRA, BRN, BRN, BRN, RED, GLW, RED, RED, GLW, RED, BRN, BRN, BRN, SNR,
TRA, TRA, BRN, BRN, BRN, RED, RED, RED, RED, RED, RED, BRN, BRN, BRN, SNR,
TRA, TRA, TRA, BRN, BRN, RED, RED, RED, RED, RED, RED, BRN, BRN, SND
};
// Ground Block Tile
int16_t sprite_Block_Ground[300] = {
LBR, LGY, LGY, LGY, LGY, LGY, LGY, LGY, LGY, BLA, LBR, LGY, LGY, LGY, LGY, LBR, SNR,
LGY, LBR, LBR, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LBR, LBR, LBR, LBR, LBR, LBR, SNR,
LGY, LBR, LBR, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LGY, LBR, LBR, LBR, LBR, LBR, SNR,
LGY, LBR, LBR, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LGY, LBR, LBR, LBR, LBR, LBR, SNR,
LGY, LBR, LBR, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LGY, BLA, LBR, LBR, LBR, BLA, SNR,
LGY, LBR, LBR, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LBR, BLA, BLA, BLA, BLA, LBR, SNR,
LGY, LBR, LBR, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LGY, LGY, LGY, LGY, LGY, BLA, SNR,
LGY, LBR, LBR, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LGY, LBR, LBR, LBR, LBR, BLA, SNR,
LGY, LBR, LBR, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LGY, LBR, LBR, LBR, LBR, BLA, SNR,
LGY, LBR, LBR, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LGY, LBR, LBR, LBR, LBR, BLA, SNR,
BLA, BLA, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LGY, LBR, LBR, LBR, LBR, LBR, BLA, SNR,
LGY, LGY, BLA, BLA, LBR, LBR, LBR, LBR, BLA, LGY, LBR, LBR, LBR, LBR, LBR, BLA, SNR,
LGY, LBR, LGY, LGY, BLA, BLA, BLA, BLA, LGY, LGY, LBR, LBR, LBR, LBR, LBR, BLA, SNR,
LGY, LBR, LBR, LBR, LGY, LGY, LGY, BLA, LGY, LBR, LBR, LBR, LBR, LBR, LBR, BLA, SNR,
LGY, LBR, LBR, LBR, LBR, LBR, LBR, BLA, LGY, LBR, LBR, LBR, LBR, LBR, BLA, BLA, SNR,
LBR, BLA, BLA, BLA, BLA, BLA, BLA, LBR, LGY, BLA, BLA, BLA, BLA, BLA, BLA, LBR, SND
};
// Bullet Bill Sprite
int16_t sprite_Enemy_Bullet[300] = {
SNR,
TRA, TRA, TRA, TRA, TRA, TRA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, TRA, BLA, BLA, SNR,
TRA, TRA, TRA, TRA, BLA, BLA, BLA, LGY, LGY, LGY, LGY, LGY, BLA, TRA, LGY, LGY, SNR,
TRA, TRA, TRA, BLA, BLA, BLA, LGY, BLA, BLA, BLA, BLA, BLA, BLA, LBR, BLA, BLA, SNR,
TRA, TRA, BLA, LGY, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, LGY, BLA, BLA, SNR,
TRA, BLA, LGY, LGY, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, LBR, BLA, BLA, SNR,
BLA, LGY, BLA, LGY, BLA, BLA, BLA, BLA, BLA, LBR, LBR, BLA, BLA, LBR, BLA, BLA, SNR,
BLA, BLA, LGY, BLA, BLA, BLA, BLA, LGY, BLA, LGY, LGY, LBR, BLA, LBR, BLA, BLA, SNR,
BLA, BLA, BLA, BLA, BLA, LGY, LGY, LGY, LGY, LGY, LGY, LBR, BLA, LBR, BLA, BLA, SNR,
BLA, BLA, BLA, BLA, BLA, LGY, LGY, LGY, LGY, LGY, BLA, BLA, BLA, LBR, BLA, BLA, SNR,
TRA, BLA, BLA, BLA, BLA, BLA, LGY, LGY, LGY, BLA, BLA, BLA, BLA, LBR, BLA, BLA, SNR,
TRA, TRA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, LBR, BLA, BLA, SNR,
TRA, TRA, TRA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, LBR, BLA, BLA, SNR,
TRA, TRA, TRA, TRA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, TRA, BLA, BLA, SNR,
TRA, TRA, TRA, TRA, TRA, TRA, BLA, BLA, BLA, BLA, BLA, BLA, BLA, TRA, BLA, BLA, SND
};

// Mario walking animation (sprite array)
int16_t* anim_Mario_Walking[4] = {sprite_Mario_Walk1, sprite_Mario_Walk2, sprite_Mario_Walk3, sprite_Mario_Walk2};

// Draw any sprite!
// sp[]     -   Sprite pixel array (max size 1000)
// _x       -   X pos of upper left pixel of sprite
// _y       -   Y pos of upper left pixel of sprite
// scale    -   1x or 2x sprite scaling option
// flip     -   flip sprite on X axis option
void Draw_Sprite(int16_t sp[], int16_t _x, int16_t _y, int16_t scale, bool flip){
    int i;
    if(!flip){ // normal sprite
        int16_t x = _x;
        int16_t y = _y;
        if(scale == 1){
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x++; }        // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x;
                    y++;
                }
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x++, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                }
            }
        }
        if(scale == 2){
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x=x+2; }      // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x;
                    y=y+2;
                }    
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x++, y+1, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x++, y+1, 1, 1);
                    SPI_WRITE16(sp[i]);
                }
            }
        }
        if(scale == 3){
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x=x+3; }      // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x;
                    y=y+3;
                }    
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x++, y+2, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x++, y+2, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x++, y+2, 1, 1);
                    SPI_WRITE16(sp[i]);
                }
            }
        }
    }
    else{ // flipped sprite
        if(scale == 1){
            int16_t x = _x+12;
            int16_t y = _y;
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x--; }        // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x+12;
                    y++;
                }    
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x--, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                }
            }
        }
        if(scale == 2){
            int16_t x = _x+24;
            int16_t y = _y;
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x=x-2; }      // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x+24;
                    y=y+2;
                }    
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x--, y+1, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x--, y+1, 1, 1);
                    SPI_WRITE16(sp[i]);
                }
            }
        }
        if(scale == 3){
            int16_t x = _x+36;
            int16_t y = _y;
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x=x-3; }      // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x+36;
                    y=y+3;
                }    
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x--, y+2, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x--, y+2, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(sp[i]);
                    setAddrWindow(x--, y+2, 1, 1);
                    SPI_WRITE16(sp[i]);
                }
            }
        }
    }
}

// Erase sprite using selectable background color
void Erase_Sprite(int16_t sp[], int16_t _x, int16_t _y, int16_t scale, bool flip, int16_t bg_color){
    int i;
    if(!flip){ // normal sprite
        int16_t x = _x;
        int16_t y = _y;
        if(scale == 1){
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x++; }        // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x;
                    y++;
                }
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x++, y, 1, 1);
                    SPI_WRITE16(bg_color);
                }
            }
        }
        if(scale == 2){
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x=x+2; }      // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x;
                    y=y+2;
                }    
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x++, y+1, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x++, y+1, 1, 1);
                    SPI_WRITE16(bg_color);
                }
            }
        }
        if(scale == 3){
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x=x+3; }      // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x;
                    y=y+3;
                }    
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x++, y+2, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x++, y+2, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x++, y+2, 1, 1);
                    SPI_WRITE16(bg_color);
                }
            }
        }
    }
    else{ // flipped sprite
        if(scale == 1){
            int16_t x = _x+12;
            int16_t y = _y;
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x--; }        // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x+12;
                    y++;
                }    
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x--, y, 1, 1);
                    SPI_WRITE16(bg_color);
                }
            }
        }
        if(scale == 2){
            int16_t x = _x+24;
            int16_t y = _y;
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x=x-2; }      // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x+24;
                    y=y+2;
                }    
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x--, y+1, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x--, y+1, 1, 1);
                    SPI_WRITE16(bg_color);
                }
            }
        }
        if(scale == 3){
            int16_t x = _x+36;
            int16_t y = _y;
            for(i = 0; i < 1000; i++){
                if(sp[i] == ILI9340_Transparent){ x=x-3; }      // transparent -- draw nothing
                else if(sp[i] == ILI9340_NextRow) {             // move to next row of sprite
                    x = _x+36;
                    y=y+3;
                }    
                else if(sp[i] == ILI9340_EndSprite){            // end of sprite
                    return;
                }
                else{                                           // draw color specified in sprite pixel array
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x--, y+2, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x--, y+2, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x, y+1, 1, 1);
                    SPI_WRITE16(bg_color);
                    setAddrWindow(x--, y+2, 1, 1);
                    SPI_WRITE16(bg_color);
                }
            }
        }
    }
}

// sprite structure / object
struct Sprite{
    int16_t curr_x;         ///< X coordinate of sprite
    int16_t curr_y;         ///< Y coordinate of sprite
    int16_t size;           ///< Size of sprite (or sprite scaling)
    bool draw;              ///< Draw sprite or not
    int16_t *graphic;       ///< Pointer to sprite graphic pixel array
    bool flipped;           ///< If sprite is flipped on X-axis
    int16_t background;     ///< Sprite solid background color
};

// create new sprite
void New_Sprite(struct Sprite *o, int16_t x, int16_t y, int16_t s, int16_t *gr, bool f, int16_t bgc){
    o->curr_x = x;
    o->curr_y = y;
    o->size = s;
    o->draw = true;
    o->graphic = gr;
    o->flipped = f;
    o->background = bgc;
    // draw the new sprite once created
    Draw_Sprite(o->graphic, o->curr_x, o->curr_y, o->size, o->flipped);
}

// Change sprite's background color
void Sprite_Change_BG(struct Sprite *o, int16_t newColor){
    o->background = newColor;
}
    
// Move sprite by dx and dy, optionally change sprite graphic for animations
void Move_Sprite(struct Sprite *o, int16_t dx, int16_t dy, int16_t *newGraphic, int16_t newDir){
    Erase_Sprite(o->graphic, o->curr_x, o->curr_y, o->size, o->flipped, o->background);
    o->curr_x = o->curr_x + dx;
    o->curr_y = o->curr_y + dy;
    o->graphic = newGraphic;
    if(newDir==1)       // new direction = right
        o->flipped = false;
    else if(newDir==2)  // new direction = left
        o->flipped = true;
    Draw_Sprite(o->graphic, o->curr_x, o->curr_y, o->size, o->flipped);
}

// Similar to Move_Sprite but just changing location and redrawing
// If -1 is input for a coordinate, the current sprite coordinate will be maintained
// If newGraphaic is null, the current graphic will be maintained
void Sprite_Change_Loc(struct Sprite *o, int16_t x, int16_t y, int16_t *newGraphic){
    Erase_Sprite(o->graphic, o->curr_x, o->curr_y, o->size, o->flipped, o->background);
    if(x != -1)
        o->curr_x = x;
    if(y != -1)
        o->curr_y = y;
    if(newGraphic != NULL)
        o->graphic = newGraphic;
    Draw_Sprite(o->graphic, o->curr_x, o->curr_y, o->size, o->flipped);
}

#ifdef	__cplusplus
}
#endif


#endif	/* ILI9340_H */

