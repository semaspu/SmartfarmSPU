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
unsigned long lastSecondTime = 0;
//=================================================================================================

//=================================================================================================
// End of Default Hardware : ET-ESP32(WROVER) RS485 V3
//=================================================================================================
 
//=================================================================================================
// Moisture Sensor : SOIL MOISTURE H/T/EC MODBUS RTU
//=================================================================================================
// Red    = +5V or 24V(3.6-30VDC)
// Black  = GND
// White  = RS485:B(-)
// Yellow = RS485:A(+)
// Green  = NC
//=================================================================================================
// Holding[512] = Slave ID
// Input[0]     = Soil Temperature/100(C) -> -4000 to +8000 : -40.0(C) to +80.0(C)
// Input[1]     = Soil Moisture/100(%H)   -> 0 - 10000 : 0 - 100(%H)
// Input[2]     = Soil EC(uS/cm)          -> 0 - 20000 : 0-20000(uS/cm)
//=================================================================================================

//=================================================================================================
// Modbus Sensor : ET-Modbus RTU SHT31 Box V2/V3
//=================================================================================================
// Red   = 7-24V
// Black = GND
// White = RS485:A(+)
// Green = RS485:B(-)
//=================================================================================================
// Input[0]       = Firmware Version
// Input[1]       = SHT31 Temperature Sensor Signed 16Bit(Signed/10)
// Input[2]       = SHT31 Humidity Sensor Signed 16Bit(Signed/10)
// Input[3]       = DS18B20 Temperature Sensor Signed 16Bit(Signed/10)
// Input[4]       = BH1750 Light Sensor Signed 16Bit(Signed/10)
// Input[5:6]     = SHT31 Temperature Sensor Float 32 Bit(Big-endian)
// Input[7:8]     = SHT31 Humidity Sensor Float 32Bit(Big-endian)
// Input[9:10]    = DS18B20 Temperature Sensor Float 32 Bit(Big-endian)
// Input[11:12]   = BH1750 Light Sensor Float 32 Bit(Big-endian)
//=================================================================================================
// Holding[0]     = Slave ID
// Holding[1]     = SHT31 Temperature Adjust Signed 16Bit(Signed/10)
// Holding[2]     = SHT31 Humidity Adjust Signed 16Bit(Signed/10)
// Holding[3]     = DS18B20 Temperature Adjust Signed 16Bit(Signed/10)
// Holding[4]     = BH1750 Light Adjust Signed 16Bit(Signed/10)
// Holding[5:6]   = SHT31 Temperature Adjust Float 32 Bit(Big-endian)
// Holding[7:8]   = SHT31 Humidity Adjust Float 32Bit(Big-endian)
// Holding[9:10]  = DS18B20 Temperature Adjust Float 32 Bit(Big-endian)
// Holding[11:12] = BH1750 Light Adjust Float 32 Bit(Big-endian)
//=================================================================================================
#include <HardwareSerial.h>
//=================================================================================================
#define SerialDebug           Serial      // USB Serial(Serial0)
//=================================================================================================
#define SerialRS485_RX_PIN    26
#define SerialRS485_TX_PIN    27
#define SerialRS485           Serial2    // Serial2(IO27=TXD,IO26=RXD)

//=================================================================================================
#include <ModbusMaster.h>             // https://github.com/4-20ma/ModbusMaster
//=================================================================================================
#define amountModbus                  2
ModbusMaster nodeSensorWeather;
ModbusMaster nodeSoilMoisture;
ModbusMaster nodeSensorWeather2;
//=================================================================================================
#define modbusMoisture_SlaveID        2     // modbus Moisture : SOIL MOISTURE H/T/EC[244]
#define modbusSensor_SlaveID          247   // modbus Sensor   : ET-MODBUS RTU SHT31 & BH1750 BOX V2/V3[247]
//=================================================================================================
uint8_t read_modbus_status;

//=================================================================================================
float sht31Temperature;
float sht31Humidity;
float bh1750Lux;
//=================================================================================================
float soilTemperature;
float soilMoisture;
float soilEc;
float soilPhosphorus;
float soilPotassium;
float soilNitrogen;
float soilPh;
float soilSalinity;
float soilTDS;
//=================================================================================================
unsigned long lastGetModbusTime = 0;
int indexGetModbus =0;                                                                              
// Ture = Sensor(SHT31,BH1750), False = Moisture
//=================================================================================================
int countReboot=0,rebootTime=60;

//===============================================================================================


void setup() {    
      //===============================================================================================
      // Start of Initial Default Hardware : ET-ESP32(WROVER) RS485 V3
      //===============================================================================================  
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
      SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
      while(!SerialRS485);
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
   
      //===============================================================================================
      // Start of Config Modbus RTU RS485  
      //===============================================================================================
      nodeSensorWeather.begin(modbusSensor_SlaveID, SerialRS485);   // ET-MODBUS RTU SHT31 & BH1750 BOX V2
      nodeSoilMoisture.begin(modbusMoisture_SlaveID, SerialRS485);  // SOIL MOISTURE H/T/EC MODBUS RTU
      //===============================================================================================
      // End of Config Modbus RTU RS485  
      //===============================================================================================
    
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

void timeDisplay()
{
    //===============================================================================================
    //==========================================================================================
    // Write Digit BCD[0..F] to Segment Display
    // dig_pos : Digit Position[0..3]
    // bcd_seg : 7-Segment BCD[0..F] Code For Display
    // dot     : Dot ON/OFF(true = ON, false = OFF)
    //==========================================================================================
    //  void ET_HT16K33_7SEGMENT::writeDigitBCD(uint8_t dig_pos,// Digit For Display
    //                                        uint8_t bcd_val,// Number For Display
    //                                        boolean dot) 
    //==========================================================================================
    myTimeNow = myRTC.now();
    if((millis() - lastSecondTime) > 500ul){    // 0.5-Second
   
      //digitalWrite(LED_PIN, LedOFF);
      //ledStatus.digitalWrite(netStatusPin, HIGH);  
      display.writeDigitBCD(0,myTimeNow.hour()/10,false);
      display.writeDigitBCD(1,myTimeNow.hour()%10,UpdateTime);
      display.writeDigitBCD(2,myTimeNow.minute()/10,false);
      display.writeDigitBCD(3,myTimeNow.minute()%10,false);
      display.writeDisplayPanel0();
      lastSecondTime = millis();
      UpdateTime=!UpdateTime;

    }
    //===============================================================================================   
}

void loop() {
         
          timeDisplay();         
    //=============================================================================================
    //                            Read Modbus Sensor  
    //=============================================================================================
    if((millis() - lastGetModbusTime) > 5000ul)  // 5-Second
      {
        //=============================================================================================
        OnLed();
        //OnBuzzer();
        //=============================================================================================
        indexGetModbus = ++indexGetModbus;  
        SerialDebug.print("indexGetModbus = ");// Toggle Status Access Modbus(Sensor/Moisture)
        SerialDebug.println(indexGetModbus);
    
        //=============================================================================================
        // Start of Read Modbus Sensor  
        //=============================================================================================
        if(indexGetModbus == 1)
        {
          //===========================================================================================
          // Modbus Sensor : ET-Modbus RTU SHT31 Box V2
          //===========================================================================================
    
            SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
            while(!SerialRS485);
            nodeSensorWeather.begin(modbusSensor_SlaveID, SerialRS485); // ET-MODBUS RTU SHT31 & BH1750 BOX V2     
            read_modbus_status = nodeSensorWeather.readInputRegisters(0x0001, 4); // 4-InputRegister Read(Input[1..4]
          //===========================================================================================
          if(read_modbus_status == nodeSensorWeather.ku8MBSuccess)
          { OffLedStatus(weather); delay(100);
            //=========================================================================================
            sht31Temperature = (int16_t)nodeSensorWeather.getResponseBuffer(0)/10.0; // SHT31 Temperature   
            sht31Humidity = (int16_t)nodeSensorWeather.getResponseBuffer(1)/10.0;    // SHT31 Humidity   
            bh1750Lux = (int16_t)nodeSensorWeather.getResponseBuffer(3)/10;         // BH1750 Light   
     
            //=========================================================================================
            SerialDebug.print("SHT31 Temperature = ");
            SerialDebug.print(sht31Temperature,1);
            SerialDebug.print(" : Humidity = ");
            SerialDebug.print(sht31Humidity,1);
            SerialDebug.print(" : Light = ");
            SerialDebug.println(bh1750Lux,1);
            display.print(sht31Temperature,1);
            display.writeDisplayPanel1();
            display.print(sht31Humidity,1);
            display.writeDisplayPanel2();
            OnLedStatus(weather);
    
            //=========================================================================================
          } else OffLedStatus(weather);
        }
        else if(indexGetModbus == 2)
        {
          //===========================================================================================
          // Moisture Sensor : SOIL MOISTURE H/T/EC MODBUS RTU
          //===========================================================================================
          // Holding[512] = Slave ID
          // Input[0]     = Soil Temperature/100(C) -> -4000 to +8000 : -40.0(C) to +80.0(C)
          // Input[1]     = Soil Moisture/100(%H)   -> 0 - 10000 : 0 - 100(%H)
          // Input[2]     = Soil EC(uS/cm)          -> 0 - 20000 : 0-20000(uS/cm)
          //===========================================================================================
          
            SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
            while(!SerialRS485);
            //nodeSensorWeather.begin(modbusSensor_SlaveID, SerialRS485);// ET-MODBUS RTU SHT31 & BH1750 BOX V2
            nodeSoilMoisture.begin(modbusMoisture_SlaveID, SerialRS485);  
            read_modbus_status = nodeSoilMoisture.readHoldingRegisters(0x0000, 9);                         // 3-InputRegister Read(Input[0..2])
            
          //===========================================================================================
          if(read_modbus_status == nodeSoilMoisture.ku8MBSuccess)
          {
            //=========================================================================================
            OffLedStatus(soil);  delay(100);
            soilTemperature = (int16_t)nodeSoilMoisture.getResponseBuffer(1) / 10.0;                  // Soil Temperature/100(C) -> -4000 to +8000 : -40.0(C) to +80.0(C)
            soilMoisture = (uint16_t)nodeSoilMoisture.getResponseBuffer(0) / 10.0;                    // Soil Moisture/100(%RH)  -> 0 - 10000 : 0 - 100(%RH)
            soilEc = (uint16_t)nodeSoilMoisture.getResponseBuffer(2);                                // Soil EC(uS/cm)          -> 0 - 20000 : 0-20000(uS/cm)
            soilPh = (uint16_t)nodeSoilMoisture.getResponseBuffer(3) / 10.0;
            soilNitrogen = (uint16_t)nodeSoilMoisture.getResponseBuffer(4);
            soilPhosphorus = (uint16_t)nodeSoilMoisture.getResponseBuffer(5);
            soilPotassium = (uint16_t)nodeSoilMoisture.getResponseBuffer(6);
            soilSalinity    = (uint16_t)nodeSoilMoisture.getResponseBuffer(7);
            soilTDS = (uint16_t)nodeSoilMoisture.getResponseBuffer(8);
   
            SerialDebug.print("Soil Temperature = ");         SerialDebug.println(soilTemperature, 1);
            SerialDebug.print("Soil Moisture = ");            SerialDebug.println(soilMoisture, 1);
            SerialDebug.print("Soil EC = ");                  SerialDebug.println(soilEc, 1);
            SerialDebug.print("Soil PH = ");                  SerialDebug.println(soilPh, 1);
            SerialDebug.print("Soil Nitrogen = ");            SerialDebug.println(soilNitrogen, 1);
            SerialDebug.print("Soil Phosphorus = ");          SerialDebug.println(soilPhosphorus, 1);
            SerialDebug.print("Soil Potassium = ");           SerialDebug.println(soilPotassium, 1);
            SerialDebug.print("Soil Salinity = ");            SerialDebug.println(soilSalinity, 1);
            SerialDebug.print("Soil TDS = ");                 SerialDebug.println(soilTDS, 1);
    
            SerialDebug.println();
            SerialDebug.println();
            display.print(soilMoisture,1);
            display.writeDisplayPanel3();  
            OnLedStatus(soil);
            //=========================================================================================
          } else OffLedStatus(soil);
          //===========================================================================================
        }
    //    else if(indexGetModbus == 3)
    //    {
    //      
    //      
    //        SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
    //        while(!SerialRS485);
    //        nodeSensorWeather.begin(modbusSensor_SlaveID_in, SerialRS485);                                     // ET-MODBUS RTU SHT31 & BH1750 BOX V2
    //      
    //        read_modbus_status = nodeSensorWeather.readInputRegisters(0x0001, 2);                       // 4-InputRegister Read(Input[1..4]
    //      //===========================================================================================
    //      if(read_modbus_status == nodeSensorWeather.ku8MBSuccess)
    //      {
    //        //=========================================================================================
    //        sht31_temperature_in = (int16_t)nodeSensorWeather.getResponseBuffer(0)/10.0;                   // SHT31 Temperature   
    //        sht31_humidity_in = (int16_t)nodeSensorWeather.getResponseBuffer(1)/10.0;                      // SHT31 Humidity   
    //        //bh1750_lux = (int16_t)nodeSensorWeather.getResponseBuffer(3)/10;                          // BH1750 Light   
    //        //=========================================================================================
    //        SerialDebug.print("SHT31 Temperature = ");
    //        SerialDebug.print(sht31_temperature_in,1);
    //        SerialDebug.print(" : Humidity = ");
    //        SerialDebug.println(sht31_humidity_in,1);
    //
    //        //=========================================================================================
    //      
    //      }
    //    }
        if(indexGetModbus == amountModbus) indexGetModbus=0;   
        //=============================================================================================
        // End of Read Modbus Sensor  
        //=============================================================================================
        OffLed();  
        //OffBuzzer();
        lastGetModbusTime = millis();   
      }  
}  // End of LOOP  
