/*********************************************** 
 Library For HT16K33 Control 7 Segment Display 
 write by...ETT CO.,LTD.
 ***********************************************/

#include <Wire.h>

#include "ET_HT16K33_7SEGMENT.h"

#ifndef _BV
  #define _BV(bit) (1<<(bit))
#endif

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

static const uint8_t numbertable[] = 
{
	0x3F,     /* 0 */
	0x06,     /* 1 */
	0x5B,     /* 2 */
	0x4F,     /* 3 */
	0x66,     /* 4 */
	0x6D,     /* 5 */
	0x7D,     /* 6 */
	0x07,     /* 7 */
	0x7F,     /* 8 */
	0x6F,     /* 9 */
	0x77,     /* a */
	0x7C,     /* b */
	0x39,     /* C */
	0x5E,     /* d */
	0x79,     /* E */
	0x71,     /* F */
};

ET_HT16K33_7SEGMENT::ET_HT16K33_7SEGMENT(uint8_t _addr = 0x70) 
{
  position = 0; 
  panel0_dg = 4; 
  panel1_dg = 4;
  panel2_dg = 4;
  panel3_dg = 4;

  i2c_addr = _addr;                                                                       // HT16K33 I2C Device Address
}

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::setBrightness(uint8_t brightness) 
{
  //========================================================================================
  if(brightness > 15)
  {
    brightness = 15;                                                                      // Maximum Brightness Adjust
  }
  //========================================================================================
  Wire.beginTransmission(i2c_addr);
  Wire.write(HT16K33_CMD_BRIGHTNESS | brightness);
  Wire.endTransmission();  
  //========================================================================================
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::blinkRate(uint8_t blinkrate) 
{
  //========================================================================================
  if(blinkrate > 3) 
  {
    blinkrate = 0;                                                                        // Disable Blink Adjust
  }
  //========================================================================================
  Wire.beginTransmission(i2c_addr);
  Wire.write(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (blinkrate << 1)); 
  Wire.endTransmission();
  //========================================================================================
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::begin(void) 
{
  //========================================================================================
  Wire.begin();
  //========================================================================================
  Wire.beginTransmission(i2c_addr);
  Wire.write(0x21);                                                                       // turn on oscillator
  Wire.endTransmission();
  //========================================================================================
  blinkRate(HT16K33_BLINK_OFF);
  //========================================================================================
  setBrightness(15);                                                                      // max brightness
  //========================================================================================
  clear();                                                                                // Clear Display
  writeUpdateDisplay();
  //========================================================================================
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::writeUpdateDisplay(void) 
{
  //========================================================================================
  Wire.beginTransmission(i2c_addr);
  //========================================================================================
  Wire.write((uint8_t)0x00);                                                              // start at address $00
  //========================================================================================
  
  //========================================================================================
  // Send LSB Data Display to HT16K33
  //========================================================================================
  for (uint8_t i=0; i<4; i++) 
  {
    Wire.write(panel0buffer[i]);                                                          // Lower-LSB  
    Wire.write(panel2buffer[i]);                                                          // Upper-LSB  
  }
  //========================================================================================
  
  //========================================================================================
  // Send MSB Data Display to HT16K33
  //========================================================================================
  for (uint8_t i=0; i<4; i++) 
  {
    Wire.write(panel1buffer[i]);                                                         // Lower-MSB  
    Wire.write(panel3buffer[i]);                                                         // Upper-MSB  
  }
  //========================================================================================
  
  //========================================================================================
  Wire.endTransmission();  
  //========================================================================================
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::writeDisplayPanel0(void) 
{
  for(uint8_t i=0; i<4; i++) 
  {
    panel0buffer[i] = displaybuffer[i];                                                   // Update Display Buffer to Panel[1] Display
  }
  writeUpdateDisplay();                                                                   // Show Display
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::writeDisplayPanel1(void) 
{
  for(uint8_t i=0; i<4; i++) 
  {
    panel1buffer[i] = displaybuffer[i];                                                   // Update Display Buffer to Panel[2] Display
  }
  writeUpdateDisplay();                                                                   // Show Display
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::writeDisplayPanel2(void) 
{
  for(uint8_t i=0; i<4; i++) 
  {
    panel2buffer[i] = displaybuffer[i];                                                   // Update Display Buffer to Panel[3] Display
  }
  writeUpdateDisplay();                                                                   // Show Display
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::writeDisplayPanel3(void) 
{
  for(uint8_t i=0; i<4; i++) 
  {
    panel3buffer[i] = displaybuffer[i];                                                   // Update Display Buffer to Panel[4] Display
  }
  writeUpdateDisplay();                                                                   // Show Display
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::clear(void) 
{
  for (uint8_t i=0; i<4; i++) 
  {
    panel0buffer[i] = 0;                                                                  // Lower-LSB  
    panel1buffer[i] = 0;                                                                  // Lower-MSB  
    panel2buffer[i] = 0;                                                                  // Upper-LSB  
    panel3buffer[i] = 0;                                                                  // Upper-MSB  
  }
  position = 0;
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::clear(uint8_t panel) 
{
  for (uint8_t i=0; i<4; i++) 
  {
    switch(panel)
    {
      case 1:
        panel0buffer[i] = 0;                                                              // Lower-LSB  
      break;
      
      case 2:
        panel1buffer[i] = 0;                                                              // Lower-MSB  
      break;
      
      case 3:  
        panel2buffer[i] = 0;                                                              // Upper-LSB  
      break;
      
      case 4:  
        panel3buffer[i] = 0;                                                              // Upper-MSB  
      break;  
    }
  }
  position = 0;
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::print(char val,                                                 // Value 
                                int base)                                                 // Value Type
{
  print((long) val, base);
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::print(unsigned char val,                                        // Value 
                                int base)                                                 // Value Type
{
  print((unsigned long) val, base);
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::print(int val,                                                  // Value 
                                int base)                                                 // Value Type
{
  print((long) val, base);
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::print(unsigned int val,                                         // Value 
                                int base)                                                 // Value Type
{
  print((unsigned long) val, base);
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::print(long val,                                                 // Value 
                                int base)                                                 // Value Type
{
  printNumber(val, base);
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::print(unsigned long val,                                        // Value 
                                int base)                                                 // Value Type
{
  if(base == BYTE) 
  {
    write(val);
  }
  else 
  {
    printNumber(val, base);
  }
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::print(double val,                                               // Value 
                                int base)                                                 // Value Type(2:BIN)
{
  printFloat(val, base);                                                                  // x.yy(DEC)
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::println(void)
{
  position = 0;
} 
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::println(char val,                                               // Value 
                                  int base)                                               // Value Type
{
  print(val, base);
  println();
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::println(unsigned char val,                                      // Value 
                                  int base)                                               // Value Type
{
  print(val, base);
  println();
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::println(int val,                                                // Value 
                                  int base)                                               // Value Type
{
  print(val, base);
  println();
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::println(unsigned int val,                                       // Value 
                                  int base)                                               // Value Type
{
  print(val, base);
  println();
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::println(long val,                                               // Value 
                                  int base)                                               // Value Type
{
  print(val, base);
  println();
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::println(unsigned long val,                                      // Value 
                                  int base)                                               // Value Type
{
  print(val, base);
  println();
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::println(double val,                                             // Value 
                                  int base)                                               // Value Type(2:BIN)
{
  print(val, base);
  println();
}
//==========================================================================================

//==========================================================================================
//==========================================================================================
void ET_HT16K33_7SEGMENT::printNumber(long val,                                           // Value 
                                      uint8_t base)                                       // Value Type
{
  printFloat(val, 0, base);                                                               // 0 fracDigits Display
}
//==========================================================================================

//==========================================================================================
// print Float value to 7-Segment Display
// val          : Value of float 
// fracDigits   : number of digit frac. Display(Default : 2) : x.yy
// base         : Type of Value(Default : DEC)
//==========================================================================================
void ET_HT16K33_7SEGMENT::printFloat(double val,                                          // Value 
                                     uint8_t fracDigits,                                  // frac. Digits(x.yy)
                                     uint8_t base)                                        // Value Type(DEC)
{ 
  uint8_t numericDigits = 4;                                                              // available digits on display
  boolean isNegative = false;                                                             // true if the number is negative
  
  // is the number negative?
  if(val < 0) 
  {
    isNegative = true;                                                                    // Enable Display Negative Sign[-]
    --numericDigits;                                                                      // Reserve 1 Digit For Display Sign[-]
    val *= -1;                                                                            // Convert Negative Value to Postive Value
  }
  
  // calculate the factor required to shift all fractional digits
  // into the integer part of the number
  double toIntFactor = 1.0;
  for(int i = 0; i < fracDigits; ++i) 
  {
    toIntFactor *= base;
  }
  
  // create integer containing digits to display by applying
  // shifting factor and rounding adjustment
  uint32_t displayNumber = val * toIntFactor + 0.5;
  
  // calculate upper bound on displayNumber given
  // available digits on display
  uint32_t tooBig = 1;
  for(int i = 0; i < numericDigits; ++i) 
  {
    tooBig *= base;
  }
  
  // if displayNumber is too large, try fewer fractional digits
  while(displayNumber >= tooBig) 
  {
    --fracDigits;
    toIntFactor /= base;
    displayNumber = val * toIntFactor + 0.5;
  }
  
  // did toIntFactor shift the decimal off the display?
  if (toIntFactor < 1) 
  {
    printError();
  } 
  else 
  {
    // otherwise, display the number
    int8_t displayPos = 3;
    if (displayNumber)  //if displayNumber is not 0
    {
      for(uint8_t i = 0; displayNumber || i <= fracDigits; ++i) 
      {
        boolean displayDecimal = (fracDigits != 0 && i == fracDigits);
        writeDigitBCD(displayPos--, displayNumber % base, displayDecimal);                // Display Value
        displayNumber /= base;
      }
    }
    else 
    {
      writeDigitBCD(displayPos--, 0, false);                                              // Display : [0]
    }
  
    // display negative sign if negative
    if(isNegative) writeDigitSegment(displayPos--, 0x40);                                 // Display Negative Sign : [-]
  
    // clear remaining display positions
    while(displayPos >= 0) writeDigitSegment(displayPos--, 0x00);                         // Display Space : [ ]
  }
}
//==========================================================================================

//==========================================================================================
// Display [-][-][-][-]
//==========================================================================================
void ET_HT16K33_7SEGMENT::printError(void) 
{
  for(uint8_t i = 0; i < MAX_7SEGMENT_DIGIT; ++i) 
  {
    writeDigitSegment(i, 0x40);                                                           // [-]:(0100 0000)
  }
}
//==========================================================================================

//==========================================================================================
// Write Digit 7-Segment(Direct) to Segment Display
// dig_pos : Digit Position[0..3]
// bcd_seg : 7-Segment Direct[0x00..0xFF] Code For Display
//         : Data[7..0] : Dp-G-F-E-D-C-B-A
// dot     : Dot ON/OFF(true = ON, false = OFF)
//==========================================================================================
void ET_HT16K33_7SEGMENT::writeDigitSegment(uint8_t dig_pos,                              // Digit Position For Display
                                            uint8_t seg_val)                              // Display Value
{
  if(dig_pos > MAX_7SEGMENT_DIGIT) return;                                                // Over Digit Display
  displaybuffer[dig_pos] = seg_val;                                                       // Update Display Buffer
}
//==========================================================================================

//==========================================================================================
// Write Digit BCD[0..F] to Segment Display
// dig_pos : Digit Position[0..3]
// bcd_seg : 7-Segment BCD[0..F] Code For Display
// dot     : Dot ON/OFF(true = ON, false = OFF)
//==========================================================================================
void ET_HT16K33_7SEGMENT::writeDigitBCD(uint8_t dig_pos,                                  // Digit For Display
                                        uint8_t bcd_val,                                  // Number For Display
                                        boolean dot)                                      // Dot 7-Segment ON/OFF Dsiplay 
{
  if(dig_pos > MAX_7SEGMENT_DIGIT) return;                                                // Over Digit Display
  writeDigitSegment(dig_pos, numbertable[bcd_val] | (dot << 7));                          // Dot = D7
}
//==========================================================================================

//==========================================================================================
// Write Display Value
//==========================================================================================
size_t ET_HT16K33_7SEGMENT::write(uint8_t c) 
{
  uint8_t r = 0;
  
  if(c == '\n') position = 0;
  if(c == '\r') position = 0;
  
  if((c >= '0') && (c <= '9')) 
  {
    writeDigitBCD(position, c-'0');
    r = 1;                                                                                // 1-Byte
  }

  position++;
  return r;
}
//==========================================================================================
