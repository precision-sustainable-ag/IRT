/* IRT2019_Client_RevAT4_06.28.ino 

IRT Datalogging System - CLIENT

Purpose: to measure temperatures at the canopy, leaf and soil surface every 10 minutes,
         save data to Flash memory and send data to Server every 30 mins

Components:
- MoteinoMega microcontroller (ATMega1284P) with LoRa radio transceiver and 4Mbit Flash chip
- DS32321 Real time clock and calendar
- 3 Melexis MLX90614 Infrared Temperature sensors (1 - 90 deg FOV, 2 - 35 deg FOV)
- 12V SLA battery
- Solar charge controller
- 12V solar panel
- IRT Client PCB

Summary points:
- Client operates only during daylight hours (7am - 7pm)
- Client takes measurements from 3 IRT sensors and saves the data to the onboard 4 Mbit Flash chip 
- Each sensor takes two measurements: object and ambient temperature (data type = float)
- 3 sensors --> 1 - canopy w/ 90 deg FOV, 1 - leaf w/ 35 deg FOV, 1 - soil surface w/ 35 deg FOV
- Each sensor is uniquely addressed; communicate with MM on I2C bus
- DS3231 RTC also communicates on I2C bus
- DS3231 RTC generates an interrupt when the alarm conditions are true
- Interrupt from RTC triggers MM to wakeup from low power sleep

Program execution:
- Menu routine (allows user to input ID and time info)
- MM enters low power sleep
- When triggered MM wakes up, checks time
  if:
  - time for measurement (every 10 mins): read sensors, save data
  - time to send (minute after each 1/2 hr): listen for up to 1 minute for 
    timestamp from Server; if timestamp received, read data from Flash and send to Server
  - hour is greater than 19 (7pm), set alarm to 7am on following day  

EEPROM:
  0:
  1: replication number
  2: hundreds place of plot number
  3: tens place of plot number
  4: ones place of plot number
  5: Server ID
  6: authorization key (2 bytes)
  7:
  8: Client radio ID
  
Changes to previous revision:
- removes erasing Flash after successful data transmission, waits until necessary

Authors: Julien Han, Justin Ayres, Alondra Thompson
Last edited: July 2019

*/

//------------ Libraries --------------------------------------

#include <Wire.h>                                     // i2c functions for RTC
#include <EEPROM.h>                                   // built-in EEPROM routines
#include <avr/sleep.h>                                // sleep functions
#include <Customized_SparkFunMLX90614.h>              // for MLX90614 series IRT
#include <DS3232RTC.h>                                // RTC library (requires TimeLib.h https://github.com/PaulStoffregen/Time)
#include "SPIFlash.h"                                 // library for external Flash chip (included in tabs)
#include <RHReliableDatagram.h>                       // allows for acknowledgments b/w radio transmissions  
#include <RH_RF95.h>                                  // library for LoRa transceiver
#include <SPI.h>                                      // SPI library for communicating with Flash chip      

#include "StringMaker.h"                              // see tab; for compiling timestamp

//------------- Assign Pins and Define Static Variables --------------------------------------

#define BATT_PIN    24                                // Measure battery voltage across A0 
#define LED_PIN     15                                // onboard LED for MM
#define START_ADDR   0x00001000                       // Leaves first sector free for saving address
#define TIMEOUT 500                                  // radio timeout for listening for a transmission
#define RETRIES 10                                    // number of retries for sending a transmission
#define EEPROM_PLOT_HUNDOS 2                          // EEPROM memory location for saving hundreds place of plot number
#define EEPROM_PLOT_TENS 3                            // EEPROM memory location for saving tens place of plot number
#define EEPROM_PLOT_ONES 4                            // EEPROM memory location for saving ones place of plot number
#define EEPROM_REP  1                                 // EEPROM memory location for saving replication number (replicates are within a plot)  
#define EEPROM_SERVER_ID 5                            // EEPROM memory location for saving Server radio ID
#define EEPROM_AUTH 6                                 // EEPROM memory location for saving authorization key (verifies transmissions b/w client & server)
#define EEPROM_RADIO 8                                // EEPROM memory location for Client radio ID   
#define RADIO_FREQ  915.0                             // LoRa radio frequency
#define TxPower     20                                // in dBm, radio Tx Power (max +23, default +13)

//------------- Global Variables ---------------------------------

byte rep;                                             // replication
int plot;                                             // plot
byte radioID = 0;                                     // client radio ID
byte serverID = 0;
int authKey = 0x63FB;

IRTherm thermB;  // leaf
IRTherm thermA;  // canopy
IRTherm thermD;   // ground

SPIFlash flash(23, 0xEF30);                           // Initialize Flash object

char nodeID[] = "IRT000_0";                           // client name: 000 --> plot number, 0 --> replication; becomes name of data file on Server microSD           

int interval = 10; // minutes                         // measurement interval
tmElements_t tm;                                      // time object for RTC (holds date and time values)

uint32_t currAddress = START_ADDR;                    // current Flash address to begin saving data to
uint32_t lastSentAddr = START_ADDR - 1;               // Flash address of last value successfully sent to Server      

RH_RF95 driver;                                       // Initialize radio object
RHReliableDatagram manager(driver, radioID);          // Initialize reliable datagram method  


//*******************************************************************************
//--------------------------- Code Starts Here ----------------------------------
//*******************************************************************************

//--------------------------------------------------------------------------//
//--------------------------- Setup ----------------------------------------//
//--------------------------------------------------------------------------//
void setup() {
  
 // --- Pin Declarations --- 
    
    pinMode(LED_PIN, OUTPUT);  
    pinMode(BATT_PIN, INPUT);
    
 // --- Initializations --- 
   
    Serial.begin(115200);                               // Serial comm (Serial monitor)
    
    flash.begin();                                      // Flash functions   
    
    Wire.begin();                                       // Initialize I2C Bus
      delay(200);
    
    RTC.begin();                                        // RTC functions
    
    thermA.begin(0x5A);                                 // IRT sensors
    thermB.begin(0x5B);
    thermD.begin(0x5D);
      delay(10);

    if (!manager.init())                                // radio
      Serial.println("Radio init failed");
      manager.setRetries(RETRIES);
      manager.setTimeout(TIMEOUT);
      driver.setFrequency(RADIO_FREQ);
      driver.setTxPower(TxPower);
      Serial.println(driver.maxMessageLength());
        
  // --- Compile plot number and nodeID[] ---
   
    plot = 100*EEPROM.read(EEPROM_PLOT_HUNDOS) + 10*EEPROM.read(EEPROM_PLOT_TENS) + EEPROM.read(EEPROM_PLOT_ONES);
    char b[4];
    itoa(plot,b,10);
    nodeID[3] = b[0];
    nodeID[4] = b[1];
    nodeID[5] = b[2];
    rep = EEPROM.read(EEPROM_REP);      // read Rep number from EEPROM
    nodeID[7] = rep + 48;


  // --- Enter menu routine ---
  
     menu();
    
  // --- Clear alarm registers, set alarm ---
    
     RTC.squareWave(SQWAVE_NONE);
     RTC.setAlarm(ALM1_MATCH_MINUTES,0,0,0,0);      // set alarm values to 0
     RTC.alarm(ALARM_1);                            // deactivate Alarm 1
     RTC.alarmInterrupt(ALARM_1,false);             // deactivate Alarm 1 interrupt
    
     RTC.read(tm);                                  // read time
     RTC.alarm(ALARM_1);                            // activate Alarm 1
     RTC.alarmInterrupt(ALARM_1, true);             // activate Alarm 1 interrupt
    
     if (tm.Hour >= 19 || tm.Hour < 7) {            // if hour later than 7pm and less than 7am
       RTC.setAlarm(ALM1_MATCH_HOURS, 0, 0, 7, 0);  // set alarm to 7am 
       Serial.println("Next Measurement: 7:00a");
     }
     else {
       RTC.setAlarm(ALM1_MATCH_MINUTES, 0, byte((((tm.Minute + interval) / interval) * interval) % 60), 0, 0 );   // set alarm to next 10 minute interval
       Serial.println("Next Measurement: " + String(byte((((tm.Minute + interval) / interval) * interval) % 60)));
     }
}

//--------------------------------------------------------------------------//
//------------------------------ Loop --------------------------------------//
//--------------------------------------------------------------------------//

void loop() {
  delay(100);
  sleepNow(); // Enter sleep
  if (RTC.alarm(ALARM_1)) {   // if Alarm 1 interrupt triggered
    digitalWrite(LED_PIN,HIGH);
    Serial.println("Awake");
    RTC.read(tm);
    
    if(tm.Minute % 10 == 0){    // time to measure
      
      takeMeasurements();
      if (tm.Minute == 0 || tm.Minute == 30){      // if minutes = 0 or 30, set alarm for next minute
        RTC.setAlarm(ALM1_MATCH_MINUTES, 0, byte(tm.Minute+1), 0, 0 );  
        Serial.println("Sending at: " + String(tm.Minute+1));
      } else {
        RTC.setAlarm(ALM1_MATCH_MINUTES, 0, byte((((tm.Minute + interval) / interval) * interval) % 60), 0, 0 );
        Serial.println("Next Measurement: " + String(int((((tm.Minute + interval) / interval) * interval) % 60)));
      }
    }
    else if (tm.Minute % 30 == 1) {     // time to send
      uint8_t from = -1;
      from = listenForServer();
      if (from == -1) {
        Serial.println("Failed to Communicate with Server!");
        delay(20);
      }
      else {
        lastSentAddr = sendData(lastSentAddr + 1, currAddress - 1);     // if data sent successfully, update flash addresses
        delay(100);
        if (lastSentAddr != 4095){
          saveCurrAddress();
          delay(100);
        }
        Serial.println("Last Sent Address: " + String(lastSentAddr));
        Serial.println("Current Address: " + String(currAddress));
        delay(100);
      }
    
    RTC.read(tm);     // read clock, set next alarm as appropriate
    if (tm.Hour >= 19 || tm.Hour < 7) {
      RTC.setAlarm(ALM1_MATCH_HOURS, 0, 0, 7, 0);
      Serial.println("Next Measurement: 7:00a");
    }
    else {
      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, byte((((tm.Minute + interval) / interval) * interval) % 60), 0, 0 );
      Serial.println("Next Measurement: " + String(int((((tm.Minute + interval) / interval) * interval) % 60)));
    }
      
  }
  }
  digitalWrite(LED_PIN,LOW);
}
//--------------------------------------------------------------------------//
//------------------------------- End Loop ---------------------------------//
//--------------------------------------------------------------------------//

//**************************************************************************//
//**************** Begin User-defined Functions ****************************//
//**************************************************************************//

//-------------------- Sleep Functions ---------------------------//
void setRTCInterrupt(){
  sei();      // turn on Global Interrupt Enable

  PCMSK0 |= (1 << PCINT2);   //  set pin as PCINT
  PCICR |= (1 << PCIE0);     //  enable interrupts on vector 0 (A)
}

void clearRTCInterrupt(){
    PCICR &= (1 << PCIE0);   // detach interrupt
}

ISR(PCINT0_vect){            // interrupt service routine (what to do upon receiving interrupt)       
  sleep_disable();           // wakeup
  clearRTCInterrupt();
}


void sleepNow() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  setRTCInterrupt();
  sleep_mode();              // sleep now
}

//--------------- Take and Record Measurements -------------------//
void takeMeasurements() {
    
    // ---- calculate battery voltage ---- //
    float battV = calcBattV();
    delay(50);
    
    // ---- read time ----
    RTC.read(tm);
    
    // ---- take measurements from IRT ---- //
    float dataIRT[3][2] = {{-999, -999},
                           {-999, -999},
                           {-999, -999}};
    delay(100);
    if (thermA.read()) {
        dataIRT[0][0] = thermA.object();
        dataIRT[0][1] = thermA.ambient();
    }
    delay(100);
    if (thermB.read()) {
        dataIRT[1][0] = thermB.object();
        dataIRT[1][1] = thermB.ambient();
    }
    delay(100);

    if (thermD.read()) {
        dataIRT[2][0] = thermD.object();
        dataIRT[2][1] = thermD.ambient();
    }
    delay(50);
    
    // ---- save data ----
    String str = StringMaker::makeString(rep, battV, tm, dataIRT);
    Serial.println(str);
    for (int i = 0; i < str.length(); i++) {
      flash.writeChar(currAddress, str.charAt(i));
      delay(20);
      currAddress += 1;
      if (currAddress >= flash.getCapacity()) {   // if Flash full, erase and start writing from beginning
          eraseFlash();
          currAddress = START_ADDR;
          lastSentAddr = START_ADDR;
      }
      delay(10);
    }

    flash.writeChar(currAddress, '\n');
    delay(50);
    currAddress += 1;
    saveCurrAddress();
    delay(50);

}

//------------------Listen for Command from Server--------------------//
uint8_t listenForServer() {
  /*if (!manager.init())
    Serial.println("radio failed");
  manager.setThisAddress(radioID);*/
  Serial.println(radioID);
  Serial.println(serverID);
  Serial.println(String(tm.Minute) + ":" + String(tm.Second));
  uint8_t buf[3];
  buf[0] = 1;
  buf[1] = 1;
  //memset(buf, 0, sizeof(buf));
  uint8_t len = sizeof(buf);
  uint8_t from;

//  manager.setTimeout(TIMEOUT);
//  manager.setRetries(RETRIES);
  
  unsigned long startTime = millis();
  while (millis() < startTime + long(120000) && from != serverID) {      //In theory, should wait up to 120 seconds
    Serial.println(String(millis() - startTime));
    //delay(50);
    //if (manager.available()) {
    //Serial.println(String(buf[0]) + " " + String(buf[1]) + " : " + String(len) + " : " + String(from) + " : " + String(manager.available()));
    //delay(100);
      if (manager.recvfromAckTimeout(buf, &len, TIMEOUT, &from)) {
        Serial.println(from);
        if (len > 0) {
          Serial.println(from);
          Serial.println(len);
        }
        Serial.println(buf[0]);
        Serial.println(buf[1]);
        tm.Minute = buf[0];
        tm.Second = buf[1];
        RTC.write(tm);
        if (from == serverID)
          return from;
      }
    //}
  }
  return -1;
  
}


//------------------Send Data over Radio -----------------------
 // Returns last address sent
 
uint32_t sendData(uint32_t startAddr, uint32_t endAddr) {
  Serial.println(endAddr);
  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  
  /* Predefined header to be sent with every message */
  data[0] = (uint8_t)(plot / 100) + 48;
  data[1] = (uint8_t)((plot % 100) / 10) + 48;
  data[2] = (uint8_t)(plot % 10) + 48;
  data[3] = '_';
  
  int i = 4;
  //flash.wakeup();
  while (startAddr <= endAddr) {
    data[i] = flash.readByte(startAddr);
    Serial.print(char(data[i]));    
    i++;delay(5);
    if (i >= RH_RF95_MAX_MESSAGE_LEN) {
      Serial.println("\nSending");
      //delay(1000);
//      delay(300);
      if (sendDataAux(data, i)) {
        i = 4;
        memset(data, 0, sizeof(data));
        delay(100);
        data[0] = (uint8_t)(plot / 100) + 48;
        data[1] = (uint8_t)((plot % 100) / 10) + 48;
        data[2] = (uint8_t)(plot % 10) + 48;
        data[3] = '_';

      }
      else
        return startAddr - (i-4) - 1;
    }
    startAddr++;
  }

  Serial.println("Done Data");
//  delay(1000);
  if (sendDataAux(data, i)) {
    data[5] = '#';
    delay(50);     // CHANGED TO 50 FROM 100 7/18 8:18
    if (sendDataAux(data, 6)) {         // send message to Server that data transfer complete
//      eraseFlash();
      delay(50);
      Serial.println("Done sending data");
//      return 4095;
    }
    delay(50);
    return endAddr;
  }
  else {
    delay(50);
    return startAddr - (i-5) - 1;
  }
}

bool sendDataAux(uint8_t *data, byte len) {
  if (manager.sendtoWait(data, len, serverID)) {
    Serial.println("Data Successfully Sent");
    delay(100);
    return true;
    // Now wait for a reply from the server
    /*uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (manager.recvfromAckTimeout(buf, &len, TIMEOUT, &from)) {
      Serial.println("Data Successfully Sent");
      Serial.println(buf[0]);
      Serial.println(buf[1]);
      tm.Minute = buf[0];
      tm.Second = buf[1];
      RTC.write(tm);
      return true;
    }
    else {
      Serial.println("No Respone From Server");
      return false;
    }*/
  }
  else
    Serial.println("Send Failed");
  delay(250);
  return false;
}

// ------------------------ Sync Server -----------------------//
void syncServer() {
  if (!manager.init())
    Serial.println("radio failed");
//  manager.setThisAddress(radioID);

Serial.println(serverID);
  
  uint8_t len = 8;
  uint8_t data[len];
  
  uint8_t from = -1;
  uint8_t buf[10];
//  memset(buf, 0, sizeof(buf));
//  delay(100);
//  memset(data, 0, sizeof(data));
//  delay(50);
  data[0] = plot / 100 + 48;
  data[1] = (plot % 100) / 10 + 48;
  data[2] = plot % 10 + 48;
  data[3] = '_';
  data[4] = rep + 48;
  data[5] = radioID;
  data[6] = (byte)((authKey >> 8) & 0xFF);
  data[7] = (byte)(authKey & 0xFF);
  delay(100);
  
  //In theory, should wait 5 seconds
  Serial.println("pinging server...");
  manager.sendtoWait(data, len, serverID);
//  delay(TIMEOUT);

  uint32_t startTime = millis();
  while(millis() <= startTime + 5000) {
    if (manager.available()) {
      if (manager.recvfromAckTimeout(buf, &len, TIMEOUT, &from)) {
        Serial.println(from);
        if (len > 0) {
          Serial.println(from);
          Serial.println(len);
        }
        /*(Serial.println(buf[0]);
        Serial.println(buf[1]);
        Serial.println(buf[2]);
        Serial.println(buf[3]);
        Serial.println(buf[4]);
        Serial.println(buf[5]);*/
        tm.Month = buf[0];
        tm.Day = buf[1];
        tm.Year = buf[2];
        tm.Hour = buf[3];
        tm.Minute = buf[4];
        tm.Second = buf[5];
        RTC.write(tm);
        if (from == serverID)
          return;
      }
    }
  }
  Serial.println("Sync Failed");
}

//--------------- Calculate Battery Voltage -------------------//
float calcBattV() {
  float R1 = 30000;                                     // resistor values for voltage divider used to calculate battery V 
  float R2 = 11000;
  float X = .00322;
  int Aout = analogRead(BATT_PIN); // analogRead returns an integer between 0 and 1023
  float Vout = Aout * X;
  return Vout * ((R1 + R2) / R2);
}

//------------- Reset function --------------------------------//
void(* resetFunc) (void) = 0;

//---------------- Erase Flash Memory -------------------------//
// Does not erase Sector 0 containing address information
void eraseFlash() {
  Serial.println("Erasing, Please Wait...");
  delay(100);
  uint32_t startTime = millis();
  for (int i = 1; i < flash.getMaxPage() / 16; i++) {
  //for (int i = 16; i < flash.getMaxPage(); i++) {
    if (startTime + 20000 <= millis()) {
      Serial.println("Timeout");
      break;
    }
    if (flash.readByte(uint32_t(4096) * uint32_t(i)) != 255) {
      Serial.println(uint32_t(4096) * uint32_t(i));
      flash.eraseSector(uint32_t(4096) * uint32_t(i));
      delay(100);
    }
    else
      break;
  }
  currAddress = START_ADDR;
  lastSentAddr = START_ADDR - 1;
  saveCurrAddress();
  Serial.println("Erasing Complete");
  delay(100);
}

//---------------- Save and Get Current Address from EEPROM (big endian) ----------------------------//
void saveCurrAddress() {
  flash.eraseSector(0, 0);
  delay(100);
  flash.writeByte(1, 0x01); // indicates to use saved addresses
    delay(15);
  flash.writeByte(2, currAddress & 0xFF);
    delay(15);
  flash.writeByte(3, (currAddress >> 8) & 0xFF);
    delay(15);
  flash.writeByte(4, (currAddress >> 16) & 0xFF);
    delay(15);
  flash.writeByte(5, (currAddress >> 24) & 0xFF);
    delay(15);
  flash.writeByte(6, lastSentAddr & 0xFF);
    delay(15);
  flash.writeByte(7, (lastSentAddr >> 8) & 0xFF);
    delay(15);
  flash.writeByte(8, (lastSentAddr >> 16) & 0xFF);
    delay(15);
  flash.writeByte(9, (lastSentAddr >> 24) & 0xFF);
    delay(15);
}

void recallCurrAddress() {
  byte buf[4];
  buf[3] = flash.readByte(2); delay(15);
  buf[2] = flash.readByte(3); delay(15);
  buf[1] = flash.readByte(4); delay(15);
  buf[0] = flash.readByte(5); delay(15);
  currAddress = ((buf[3] << 0) & 0xFF) + ((buf[2] << 8) & 0xFFFF) + ((((uint32_t)buf[1]) << 16) & 0xFFFFFF) + ((((uint32_t)buf[0]) << 24) & 0xFFFFFFFF);
    delay(50);
  buf[3] = flash.readByte(6); delay(15);
  buf[2] = flash.readByte(7); delay(15);
  buf[1] = flash.readByte(8); delay(15);
  buf[0] = flash.readByte(9); delay(15);
  lastSentAddr = ((buf[3] << 0) & 0xFF) + ((buf[2] << 8) & 0xFFFF) + ((((uint32_t)buf[1]) << 16) & 0xFFFFFF) + ((((uint32_t)buf[0]) << 24) & 0xFFFFFFFF);
  delay(50);
}

void printFlashData() { 
  for (uint32_t addr = 4096; addr < flash.getCapacity(); addr++) {
    byte b = flash.readByte(addr);
    if (b == 255)
      return;
    Serial.print(char(b));
    delay(10);
  }
}

//**************************************************************************************************************************//
//**************************************************************************************************************************//

//-------------------- Menu --------------------------------------
void menu()
{
  if (Serial.available() > 0)
    Serial.read();

  plot = 100*EEPROM.read(EEPROM_PLOT_HUNDOS) + 10*EEPROM.read(EEPROM_PLOT_TENS) + EEPROM.read(EEPROM_PLOT_ONES);    // construct plot number
  char b[4];
  itoa(plot,b,10);
  nodeID[3] = b[0];                       // construct nodeID with plot number
  nodeID[4] = b[1];
  nodeID[5] = b[2];

  rep = EEPROM.read(EEPROM_REP);          // read Rep number from EEPROM
  nodeID[7] = rep + 48;

  radioID = EEPROM.read(EEPROM_RADIO);    // read radio ID from EEPROM
  manager.setThisAddress(radioID);        // set radio address
//  manager.setRetries(RETRIES);            // set number fo retries    

  serverID = EEPROM.read(EEPROM_SERVER_ID);     // read server ID from EEPROM
  authKey = ((EEPROM.read(EEPROM_AUTH + 1) << 8) & 0xFFFF) + ((EEPROM.read(EEPROM_AUTH)) & 0xFF);     // construct authorization key
  
  //--- Print header ---

  Serial.println();
  Serial.println(F("IRT Client Datalogger"));
  Serial.print(F("Client ID: "));
  Serial.println(radioID);
  Serial.print(F("Server ID: "));
  Serial.println(serverID);
  Serial.print(F("Authentication Key: "));
  Serial.println(authKey);

  Serial.println();                       // print out board info
  Serial.print(F("Plot: "));
  Serial.println(plot);
  Serial.print(F("Rep: "));
  Serial.println(rep);
  Serial.print(F("Radio ID: "));
  Serial.println(radioID);


  Serial.print(F("Data nodeID: "));        // SDcard nodeID
  Serial.println(nodeID);

  RTC.read(tm);
  Serial.print(F("Current date:  "));
  Serial.print(tm.Month);                  // date
  Serial.print('-');
  Serial.print(tm.Day);
  Serial.print('-');
  Serial.println(tm.Year + 1970);

  Serial.print(F("Current time:  "));
  Serial.print(tm.Hour);                   // time
  Serial.print(':');
  if (tm.Minute < 10)
  {
    Serial.print('0');
  }
  Serial.print(tm.Minute);
  Serial.print(':');
  if (tm.Second < 10)
  {
    Serial.print('0');
  }
  Serial.println(tm.Second);

  if (flash.readByte(1) == 0x01)            // recall current address in Flash to which to begin saving data
    recallCurrAddress();

  Serial.println("\nCurrent Address: " + String(currAddress));
  Serial.println("Last Sent to Server: " + String(lastSentAddr) + "\n");

  //--- Menu options ---
  Serial.println("Menu options: ");
  Serial.println(F("   a  <--  Sync with Server"));
  Serial.println(F("   c  <--  Set Clock"));
  Serial.println(F("   i  <--  Set Plot, Rep Number, and Radio ID"));
//  Serial.println(F("   k  <--  Enter Authentication Key"));
  Serial.println(F("   s  <--  Enter Server ID"));
  Serial.println(F("   t  <--  Test Measurements"));
  Serial.println(F("   p  <--  Print data to window"));
  Serial.println(F("   e  <--  Erase Flash Memory"));
  Serial.println(F("   x  <--  Exit"));

  // wait 20 secs for input
  int timeout = millis() + 20000;
  int menuinput = -1;
  while (millis() < timeout)
  {
    //int menuinput = 120;
    if (Serial.available() > 0) {                       // If something typed, go to menu
      menuinput = Serial.read();                        // Get input from Serial Monitor
      while (Serial.available() > 0)
        Serial.read();

      break;
    }
  }
  int eraseInput = -1;
  
  switch (menuinput) {
    case 97:  // -------------- a --------------------
      Serial.println("Attempting to Sync With Server");
      delay(50);
      syncServer();
      menu();
      break;
    case 99:  // -------------- c --------------------

      Serial.println(F("Set clock: "));
      delay(30);

      Serial.print(F("  input month:  "));
      tm.Month = getInput(timeout);
      Serial.print(F("  input day:    "));
      tm.Day = getInput(timeout);
      Serial.print(F("  input year:   "));
      tm.Year = getInput(timeout) - 1970;               // Years since 1970
      Serial.print(F("  input hour:   "));
      tm.Hour = getInput(timeout);
      Serial.print(F("  input minute: "));
      tm.Minute = getInput(timeout);
      //secs = 0;

      RTC.write(tm);
      delay(100);

      menu();
      break;

    case 101:  // -------------- e --------------------
      Serial.println(F("  Are you sure you wish to erase all flash memory? (y/n): "));
      delay(100);
      eraseInput = getCharInput(timeout);
      if (eraseInput == 121) {
        eraseFlash();
      }
      menu();
      break;

    case 105:  // -------------- i --------------------
      Serial.println(F("Set Plot Number, Rep, and Radio ID:"));
      Serial.flush();

      Serial.print(F(" Plot:     "));               // get Plot number
      Serial.flush();                               // decode user input
      plot = getInput(timeout);

      Serial.print(F(" Rep:  "));                   // get radioID
      Serial.flush();
      rep = getInput(timeout);
      
      Serial.print(F(" Radio ID:  "));                   // get radioID
      Serial.flush();
      radioID = byte(getInput(timeout));
      EEPROM.write(EEPROM_RADIO, radioID);

      /*hundred = plot / 100;
      num = plot % 100;
      radioID = 32 * (hundred - 1) + 2 * num + rep;*/
      manager.setThisAddress(radioID);


      EEPROM.write(EEPROM_PLOT_HUNDOS, byte(plot/100));                           // store settings to EEPROM
      delay(20);
      EEPROM.write(EEPROM_PLOT_TENS, byte((plot%100)/10));                        // store settings to EEPROM
      delay(20);
      EEPROM.write(EEPROM_PLOT_ONES, byte(plot%10));                              // store settings to EEPROM
      delay(20);
      EEPROM.write(EEPROM_REP, rep);
      delay(20);

      resetFunc();//menu();                                       // go back to menu
      break;

   /* case 107:  // -------------- k --------------------
      Serial.println(F("Enter Authentication Key:"));
      authKey = getInput(timeout);
      Serial.println(authKey);
      Serial.println((byte)(authKey & 0xFF));
      Serial.println((byte)((authKey >> 8) & 0xFF));
      EEPROM.write(EEPROM_AUTH, (byte)(authKey & 0xFF));
      EEPROM.write(EEPROM_AUTH + 1, (byte)((authKey >> 8) & 0xFF));
      delay(20);

      resetFunc();//menu();                                       // go back to menu
      break;*/

    case 112:  // -------------- p --------------------
      printFlashData();
      resetFunc();
      break;

    /*case 114:  // -------------- r --------------------
      Serial.println("Start Erasing");
      delay(100);
      flash.eraseChip();
      delay(100);
      Serial.println("Done Erasing");
      delay(100);
      resetFunc();
      break;*/

    case 115:  // -------------- s --------------------
      Serial.println(F("Enter Server ID:"));
      Serial.flush();
      
      serverID = getInput(timeout);

      EEPROM.write(EEPROM_SERVER_ID, serverID);                           // store settings to EEPROM
      delay(20);

      resetFunc();//menu();                                       // go back to menu
      break;

    case 116:  // -------------- t --------------------
      Serial.println(F("Test measurements:"));      // take 3 readings to check sensors
      Serial.println();
      delay(10);

      if (thermD.read()) {
        Serial.print(String(thermD.object()) + ",");
        Serial.print(String(thermD.ambient()) + ",");
      }
      else {
        Serial.print("-,-,");
      }

      delay(100);
      if (thermB.read()) {
        Serial.print(String(thermB.object()) + ",");
        Serial.print(String(thermB.ambient()) + ",");
      }
      else {
        Serial.print("-,-,");
      }

      delay(100);
      if (thermA.read()) {
        Serial.print(String(thermA.object()) + ",");
        Serial.println(String(thermA.ambient()));
      }
      else {
        Serial.print("-,-,");
      }

      delay(50);

      menu();
      break;

    case 120:  // -------------- x --------------------
      Serial.println(F("Exit"));                           // exit
      Serial.println();
      delay(10);

      break;                                               // exit switch(case) routine

    default:  // -------------- default --------------------
      Serial.println(F("Exit"));                           // exit
      Serial.println();
      delay(10);

      break;                                               // if no valid user input, leave menu
  }

  delay(100);
}

//------------- Get User Input --------------------------------------

int getInput(int timeout)
{
  timeout = millis() + 10000;         // time to wait for user to input something

  int indata = 0;                                       // initialize
  while (millis() < timeout)                            // wait for user to input something
  {
    if (Serial.available() > 0)                         // something came in to BT serial buffer
    {
      delay(100);                                       // give time for everything to come in
      int numincoming = Serial.available();             // number of incoming bytes
      for (int i = 1; i <= numincoming; i++)                // read in everything
      {
        int incoming[64];
        incoming[i] = Serial.read();                    // read from buffer
        if (incoming[i] != 13 && incoming[i] != 10) {   // ignore CR or LF
          int input = incoming[i] - 48;                     // convert ASCII value to decimal
          indata = indata * 10 + input;                 // assemble to get total value
        }
      }
      break;                                            // exit before timeout
    }
  }
  Serial.println(indata);
  return indata;
}

int getCharInput(int timeout)
{
  timeout = millis() + 10000;         // time to wait for user to input something

  int indata = 0;                                       // initialize
  while (millis() < timeout)                            // wait for user to input something
  {
    if (Serial.available() > 0)                         // something came in to BT serial buffer
    {
      delay(100);                                       // give time for everything to come in

      indata = Serial.read();                    // read from buffer
      break;
    }
  }
  Serial.println(indata);
  return indata;   //return first character
}
