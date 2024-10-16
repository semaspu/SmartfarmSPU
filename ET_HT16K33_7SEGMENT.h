/*********************************************** 
 Library For HT16K33 Control 7 Segment Display 
 write by...ETT CO.,LTD.
 ***********************************************/
#ifndef ET_HT16K33_7SEGMENT_h
#define ET_HT16K33_7SEGMENT_h

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

 #include <Wire.h>

#define HT16K33_BLINK_CMD 			  0x80
#define HT16K33_BLINK_DISPLAYON 	0x01
#define HT16K33_BLINK_OFF 			  0
#define HT16K33_BLINK_2HZ  			  1
#define HT16K33_BLINK_1HZ  			  2
#define HT16K33_BLINK_HALFHZ  		3

#define HT16K33_CMD_BRIGHTNESS 		0xE0

#define MAX_7SEGMENT_DIGIT		    4

#define HEX                       16
#define DEC                       10
#define OCT                       8
#define BIN                       2
#define BYTE                      0

class ET_HT16K33_7SEGMENT : public Print
{
 public:
  ET_HT16K33_7SEGMENT(uint8_t _addr);
  
  void begin(void);                           // I2C Address
  void setBrightness(uint8_t brightness);     // Setup Brightness
  void blinkRate(uint8_t blinkrate);          // Setup Blink Rate  
  void writeUpdateDisplay(void);              // Update Display Panel[1..4]
  void writeDisplayPanel0(void);              // Update Display Panel[1]
  void writeDisplayPanel1(void);              // Update Display Panel[2]
  void writeDisplayPanel2(void);              // Update Display Panel[3]
  void writeDisplayPanel3(void);              // Update Display Panel[4]
  void clear(void);                           // Clear All Display
  void clear(uint8_t panel);                  // Clear Panel[n] Display
  //
  size_t write(uint8_t c);
  
  void print(char val,                        // Value 
             int base = BYTE);                // Value Type
             
  void print(unsigned char,                   // Value
             int base = BYTE);                // Value Type
             
  void print(int val,                         // Value
             int base = DEC);                 // Value Type
             
  void print(unsigned int val,                // Value
             int base = DEC);                 // Value Type
              
  void print(long val,                        // Value  
             int base = DEC);                 // Value Type
             
  void print(unsigned long val,               // Value 
             int base = DEC);                 // Value Type
             
  void print(double val,                      // Value
             int base = BIN);                 // Value Type(2)
  
  void println(void);
  void println(char val,                      // Value
               int base = BYTE);              // Value Type
               
  void println(unsigned char val,             // Value
               int base = BYTE);              // Value Type
               
  void println(int val,                       // Value
               int base = DEC);               // Value Type
               
  void println(unsigned int val,              // Value
               int base = DEC);               // Value Type
               
  void println(long val,                      // Value
               int base = DEC);               // Value Type
               
  void println(unsigned long val,             // Value
               int base = DEC);               // Value Type
               
  void println(double val,                    // Value
               int base = BIN);               // Value Type(2)
               
  void printNumber(long val,                  // Value
                   uint8_t = BIN);            // Value Type(2)
                   
  void printFloat(double val,                 // Value
                  uint8_t fracDigits = 2,     // frac. Digits(x.yy)
                  uint8_t base = DEC);        // Value Type
                  
  void printError(void);
  
  void writeDigitSegment(uint8_t dig_pos,     // Digit Position For Display
                         uint8_t seg_val);    // 7-Segment Value
                     
  void writeDigitBCD(uint8_t dig_pos,         // Digit Position For Display
                     uint8_t bcd_val,         // BCD[0..F] For Display
                     boolean dot = false);    // Dot 7-Segment ON/OFF Display
 
 private:
  uint8_t position;                           // Dislay Digit Position
  uint8_t displaybuffer[4];                   // Dummy Display Buffer
  uint8_t panel0buffer[4];                    // Panel[0] Display Buffer(Lower LSB)
  uint8_t panel1buffer[4];                    // Panel[1] Display Buffer(Lower MSB)
  uint8_t panel2buffer[4];                    // Panel[2] Display Buffer(Upper LSB)
  uint8_t panel3buffer[4];                    // Panel[3] Display Buffer(Upper MSB)
  
 protected:
  uint8_t i2c_addr;                           // HT16K33 I2C Address
  
  uint8_t panel0_dg;                          // Panel[0] Digit Display
  uint8_t panel1_dg;                          // Panel[1] Digit Display
  uint8_t panel2_dg;                          // Panel[2] Digit Display
  uint8_t panel3_dg;                          // Panel[3] Digit Display
};

#endif 
