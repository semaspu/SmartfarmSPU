//=================================================================================================
// Start of Default Hardware : ET-ESP32(WROVER) RS485 V3
//=================================================================================================
// Remap Pin USART -> 
//C:\Users\Admin\Documents\Arduino\hardware\espressif\esp32\cores\esp32\HardwareSerial.cpp
//C:\Users\Admin\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.0\cores\esp32\HardwareSerial.cpp
//=================================================================================================
#include <ET_HT16K33_7SEGMENT.h>
#define I2C_SCL_PIN           22        // ESP32-WROVER : IO22(SCL1)
#define I2C_SDA_PIN           21        // ESP32-WROVER : IO21(SDA1)
//=================================================================================================
#define LED_PIN               2         // ESP32-WROVER  : IO2
#define LedON                 HIGH
#define LedOFF                LOW
#define InitialLed()          pinMode(LED_PIN,OUTPUT)
#define OnLed()               digitalWrite(LED_PIN, LedON)
#define OffLed()              digitalWrite(LED_PIN, LedOFF)
//================================================================================================= 
#define UserSwitchPin         36        // ESP32-WROVER :IO36
#define SwLogicPress          LOW
#define SwLogicRelease        HIGH 
#define InitialUserSwitch()   pinMode(UserSwitchPin,INPUT_PULLUP)
//=================================================================================================
#define BUZZER_PIN            32
#define InitialBuzzer()       pinMode(BUZZER_PIN,OUTPUT)// ESP32-WROVER :IO32
#define BUZZER_OFF            LOW
#define BUZZER_ON             HIGH 
#define OnBuzzer()            digitalWrite(BUZZER_PIN, BUZZER_ON) 
#define OffBuzzer()           digitalWrite(BUZZER_PIN, BUZZER_OFF)
//=================================================================================================
#define OptoOutputPin         25        // ESP32-WROVER : IO25
#define OptoOutputLogicOn     LOW
#define OptoOutputLogicOff    HIGH
#define InitialOptoOutput()   pinMode(OptoOutputPin,OUTPUT)
#define OnOptoOutput()        digitalWrite(OptoOutputPin, OptoOutputLogicOn)
#define OffOptoOutput()       digitalWrite(OptoOutputPin, OptoOutputLogicOff)
//=================================================================================================
// ET_DS3231.h
//=================================================================================================
#include <ET_DS3231.h>
//=================================================================================================
ET_DS3231 myRTC;
DateTime myTimeNow;
int Hour, Minute, Secund, Year, Month, Day;
//=================================================================================================
ET_HT16K33_7SEGMENT display = ET_HT16K33_7SEGMENT(0x70);
//=================================================================================================
#include <ETT_PCF8574.h>
//=================================================================================================
#define   addLedStatus        0x3A      //011 1001+(0:W,1:R) 
#define   addKeyLedStatus     0x3B      //011 1011+(0:W,1:R) 
#define   addRelayStatus      0x38      //011 1000+(0:W,1:R)  0x38+1=39

ETT_PCF8574 exp_i2c_io(addRelayStatus); // ET-ESP32-RS485 V3 : Input/Output(PCF8574:ID0)
//=================================================================================================
#define ry0_onboard_pin       0           // P0 = Relay[0]
#define ry1_onboard_pin       2           // P2 = Relay[1]
#define ry2_onboard_pin       4           // P4 = Relay[2]
#define ry3_onboard_pin       6           // P6 = Relay[3]
//=================================================================================================
#define RelayOn               LOW         // On Relay Ative(LOW)
#define RelayOff              HIGH        // Off Relay Active(LOW)
//=================================================================================================
int relay_onboard_pin[4] =    {ry0_onboard_pin, ry1_onboard_pin, 
                               ry2_onboard_pin, ry3_onboard_pin};// ett relay/pcf8574
//=================================================================================================
#define opto_in0_pin          1           // P1 Read Opto In(0) Active(LOW)
#define opto_in1_pin          3           // P3 Read Opto In(1) Active(LOW)
#define opto_in2_pin          5           // P5 Read Opto In(2) Active(LOW)
#define opto_in3_pin          7           // P7 Read Opto In(3) Active(LOW)
//=================================================================================================
#define OptoLogicOn           LOW         // Opto In On(LOW)
#define OptoLogicOff          HIGH        // Opto In Off(HIGH)
        
//=================================================================================================
int opto_input_pin[4] = {opto_in0_pin, opto_in1_pin, opto_in2_pin, opto_in3_pin};                 // ett opto-in/pcf8574
//=================================================================================================

//=================================================================================================
// Read Opto by PCF8574 Pin
//=================================================================================================
#define InitialOptoInput(i)   exp_i2c_io.writePin(opto_input_pin[i], HIGH)// Initial Input Pin
#define ReadOptoInput(i)      !exp_i2c_io.readPin(opto_input_pin[i])  // Read Opto Input : 0 = OFF, 1 = ON
//=================================================================================================  

//=================================================================================================
// Control Relay by PCF8574 Pin
//=================================================================================================
#define ReadRelay(r)          !exp_i2c_io.readPin(relay_onboard_pin[r])// Read Relay ON(1)/OFF(0) Status
#define OnRelay(r)            exp_i2c_io.writePin(relay_onboard_pin[r], RelayOn)                  // Open  = ON  Relay
#define OffRelay(r)           exp_i2c_io.writePin(relay_onboard_pin[r], RelayOff)                 // Close = OFF Relay
#define ToggleRelay(r)        exp_i2c_io.writePin(relay_onboard_pin[r], !exp_i2c_io.readPin(relay_onboard_pin[r]))   // Close = OFF Relay
//=================================================================================================

//=================================================================================================
ETT_PCF8574 exp_i2c_led(addLedStatus); // ET-ESP32-RS485 V3 : Input/Output(PCF8574:ID0)
//=================================================================================================
#define   wifi              4 
#define   link              5 
#define   weather           6
#define   soil              7 


#define   turnOn              LOW
#define   turnOff             HIGH

#define OnLedStatus(r)        exp_i2c_led.writePin(r, turnOn)                  // Open  = ON  Relay
#define OffLedStatus(r)       exp_i2c_led.writePin(r, turnOff)                 // Close = OFF Relay
#define ToggleLedStatus(r)    exp_i2c_led.writePin(r, !exp_i2c_led.readPin(r))   // Close = OFF Relay


//=================================================================================================
ETT_PCF8574 exp_i2c_SwLed(addKeyLedStatus); // ET-ESP32-RS485 V3 : Input/Output(PCF8574:ID0)
//=================================================================================================
#define   ledSw0             6 
#define   ledSw1             4 
#define   ledSw2             2 
#define   ledSw3             0 
int ledSw_onboard_pin[4] =    {ledSw0, ledSw1,ledSw2, ledSw3};

#define   SW0                7 
#define   SW1                5 
#define   SW2                3 
#define   SW3                1 
int sw_onboard_pin[4] =    {SW0, SW1,SW2, SW3};

#define   turnOn              LOW
#define   turnOff             HIGH
#define   keypress            HIGH
#define   keyrelease          LOW

#define   OnLedSw(r)            exp_i2c_SwLed.writePin(ledSw_onboard_pin[r], turnOn)// Open  = ON  LED
#define   OffLedSw(r)           exp_i2c_SwLed.writePin(ledSw_onboard_pin[r], turnOff)// Close = OFF LED
#define   ToggLedSw(r)          exp_i2c_SwLed.writePin(ledSw_onboard_pin[r], !exp_i2c_SwLed.readPin(ledSw_onboard_pin[r]))    
// Close = OFF Relay

//=================================================================================================
#define InitialSwitch(i)   exp_i2c_SwLed.writePin(sw_onboard_pin[i], HIGH)// Initial Input Pin
#define ReadSwitch(i)      !exp_i2c_SwLed.readPin(sw_onboard_pin[i])  // Read Opto Input : 0 = OFF, 1 = ON
//=================================================================================================


//=================================================================================================
float UTCOffset = +7.0;    // Your timezone relative to UTC (http://en.wikipedia.org/wiki/UTC_offset)
char daysOfTheWeek[8][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
//=================================================================================================
bool UpdateTime = false;   

#include <HardwareSerial.h>
//=================================================================================================
#define SerialDebug           Serial      // USB Serial(Serial0)
//=================================================================================================
#define SerialRS485_RX_PIN    26
#define SerialRS485_TX_PIN    27
#define SerialRS485           Serial2    // Serial2(IO27=TXD,IO26=RXD)

//===============================================================================================


void setup() {


      InitialLed();
      InitialUserSwitch();
      InitialBuzzer();
      InitialOptoOutput();    

      //===============================================================================================
      Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);                                                      
      //===============================================================================================
      //===============================================================================================
      Serial.begin(115200);
      SerialDebug.begin(115200);
      while(!SerialDebug);
     
      //===============================================================================================
      // End of Initial Default Hardware : ET-ESP32(WROVER) RS485 V2
      //===============================================================================================

      //===============================================================================================
      exp_i2c_io.begin(0xFF);
      //===============================================================================================
      //===============================================================================================
      OffRelay(0);      OffRelay(1);      OffRelay(2);      OffRelay(3);    
      OffLedSw(0);      OffLedSw(1);      OffLedSw(2);      OffLedSw(3);  
      InitialOptoInput(0);  InitialOptoInput(1);  InitialOptoInput(2);  InitialOptoInput(3);
      InitialSwitch(0);     InitialSwitch(1);     InitialSwitch(2);     InitialSwitch(3);     
      
      SerialDebug.println("ET-ESP32(WROVER)RS485 V3.....Start");
      //===============================================================================================
      SerialDebug.println("ET-ESP32(WROVER)RS485 V3.....Ready");
      SerialDebug.println();
      //===============================================================================================
      SerialDebug.println();
      SerialDebug.println("================================================================");
      SerialDebug.println("Initialize DS3231");
      //===============================================================================================
      myRTC.begin();
    //===============================================================================================
      if(myRTC.lostPower()) 
      {
        SerialDebug.println("RTC lost power, lets set the time!");
        myRTC.adjust(DateTime(F(__DATE__), F(__TIME__)));                                             // Setup RTC from date & time this sketch was compiled
      }
      //myRTC.adjust(DateTime(F(__DATE__), F(__TIME__)));  
    //===============================================================================================
    
    //===============================================================================================
    myRTC.armAlarm1(false);
    myRTC.clearAlarm1();
    //===============================================================================================
    myRTC.armAlarm2(false);
    myRTC.clearAlarm2();
    //===============================================================================================
    
    //===============================================================================================
    myRTC.setAlarm1(0, 0, 0, 0, DS3231_EVERY_SECOND);                                               // Alarm Every Second
    //===============================================================================================
    //===============================================================================================
    SerialDebug.println("Initial RTC:DS3231....Complete");
    //===============================================================================================
  
    Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);  
     
    //=======================================================================================
    // Initial HT16K33 Display Control
    //=======================================================================================
    display.begin();
    //======================================================================================= 
}

void loop() {
  OnRelay(0);                  delay(1000);    //ON  Relay  and Delay 1000 ms
  OffRelay(0);                 delay(1000);    //OFF Relay  and Delay 1000 ms 
  
  OnRelay(1);                  delay(1000);    //ON  Relay  and Delay 1000 ms
  OffRelay(1);                 delay(1000);    //OFF Relay  and Delay 1000 ms 
  
  OnRelay(2);                  delay(1000);    //ON  Relay  and Delay 1000 ms
  OffRelay(2);                 delay(1000);    //OFF Relay  and Delay 1000 ms 

  OnRelay(3);                  delay(1000);    //ON  Relay  and Delay 1000 ms
  OffRelay(3);                 delay(1000);    //OFF Relay  and Delay 1000 ms 
  
}
