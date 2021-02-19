/* IRT2019_Server_RevAT3_06.28.ino 

IRT Datalogging System - SERVER

Purpose: to poll clients for data every 30 minutes, save all received client data to SD card

Components:
- MoteinoMega microcontroller (ATMega1284P) with LoRa radio transceiver and 4Mbit Flash chip
- DS32321 Real time clock and calendar
- SparkFun Transflash microSD Breakout w/ microSD card
- 12V SLA battery
- Solar charge controller
- 12V solar panel
- IRT Server PCB

Summary points:
- Server only operates during daylight hours (7am - 7pm)
- DS3231 RTC communicates on I2C bus, microSD breakout on SPI
- DS3231 RTC generates an interrupt when the alarm conditions are true
- Interrupt triggers MM to wakeup from low power sleep
- Received data are first saved to partitioned sections of Flash chip then 
  dumped daily to microSD card

Program execution:
- Menu routine (allows user to input ID info, client IDs, time info)
- MM enters low power sleep
- When triggered MM wakes up, check time
  if:
  - minute value is 1 or 31: ping each client ID saved in Flash memory with timestamp, 
    if response received, save data to corresponding section in Flash
  - hour value is 19, dump data saved in Flash to microSD card, erase data from Flash

EEPROM:
 0:
 1:
 2:
 3:
 4:
 5: serverID

Changes to previous revision:
- delay, retry, timeout values

Authors: Julien Han, Justin Ayres, Alondra Thompson
Last edited: July 2019


*/

//------------ Libraries --------------------------------------

#include <Wire.h>                                     // i2c functions for RTC
#include <EEPROM.h>                                   // built-in EEPROM routines
#include <avr/sleep.h>                                // sleep functions
#include <DS3232RTC.h>                                // RTC library (requires TimeLib.h https://github.com/PaulStoffregen/Time)
#include "SPIFlash.h"                                 // library for external Flash chip (included in tabs)
#include <RHReliableDatagram.h>                       // allows for acknowledgments b/w radio transmissions 
#include <RH_RF95.h>                                  // library for LoRa transceiver
#include <SPI.h>                                      // SPI library for communicating with Flash chip 
#include "StringMaker.h"                              // see tab; for compiling timestamp
#include "SDHandler.h"                                // see tab; for saving data to SD card

//------------- Assign Pins and Define Static Variables --------------------------------------

#define BATT_PIN    24                                // Measure battery voltage across A0 
#define LED_PIN     15                                // onboard LED for MM
#define START_ADDR  0x00001000                        // Leaves first sector free for saving address
#define TIMEOUT     500                               // radio timeout for listening for a transmission (was 1000)
#define RETRIES     10                                // number of retries for sending a transmission (was 5)
#define MAX_CLIENTS 6                                 // maximum number of networked clients
#define EEPROM_SERVER_ID 5                            // EEPROM memory location for Server radio ID
#define RADIO_FREQ  915.0                             // LoRa radio frequency
#define TxPower     20                                // in dBm, radio Tx Power (max +23, default +13)

//------------- Global Variables ---------------------------------

byte rep;                                             // replication
int plot;                                             // plot
char clientIDs[MAX_CLIENTS][10];                      // char array for storing client IDs used for naming files
byte radioIDs[MAX_CLIENTS];                           // array for storing radio IDs of networked clients
byte numClients = 0;                          
byte serverID = 0;
const int authKey = 0x63FB;

SPIFlash flash(23, 0xEF30);                           // Initialize Flash object  
SDHandler sd;                                         // Initialize SD object
File root;

tmElements_t tm;                                      // time object for RTC (holds date and time values)  

/*uint32_t currAddress = START_ADDR;
uint32_t lastSentAddr = START_ADDR - 1;*/

RH_RF95 driver;                                       // Initialize radio object
RHReliableDatagram manager(driver, serverID);         // Initialize reliable datagram method  

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

  // --- Initializations --
    
      Serial.begin(115200);                               // Serial comm (Serial monitor)
      
      flash.begin();                                      // Flash functions 
  
      Wire.begin();                                       // Initialize I2C Bus
        delay(200);
      
      RTC.begin();                                        // RTC functions
        delay(10);
     
      if (!manager.init())
        Serial.println("Radio init failed");
        manager.setRetries(RETRIES);
        manager.setTimeout(TIMEOUT);
        driver.setFrequency(RADIO_FREQ);
        driver.setTxPower(TxPower);
        
  
      if (!SD.begin(3)) {
       Serial.println("SD initialization failed.");       // blink LED if SD fails to init
       for (byte h = 0; h < 20; h++){
         digitalWrite(LED_PIN, HIGH); 
         delay(500);
         digitalWrite(LED_PIN, LOW);
        }    
      }

  // --- Enter menu routine ---
  
     menu();
    
  //--- Clear alarm registers, set alarm ---
    
    RTC.squareWave(SQWAVE_NONE);
    RTC.setAlarm(ALM1_MATCH_MINUTES,0,0,0,0);      // set alarm values to 0
    RTC.alarm(ALARM_1);                            // deactivate Alarm 1
    RTC.alarmInterrupt(ALARM_1,false);             // deactivate Alarm 1 interrupt   
    
    RTC.read(tm);                                  // read time
    RTC.alarm(ALARM_1);                            // activate Alarm 1
    RTC.alarmInterrupt(ALARM_1, true);             // activate Alarm 1 interrupt 
    
    if (tm.Hour >= 19 || tm.Hour < 7) {            // if hour later than 7pm and less than 7am
      RTC.setAlarm(ALM1_MATCH_HOURS, 0, 0, 7, 0);  // set alarm to 7am 
      Serial.println("Alarm Set for 7:00a");
    }
    else if (tm.Minute >= 31){                     // if minute > 31, set alarm for :01
/*      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, byte(((((tm.Minute + 10) / 10) * 10) % 60) + 1), 0, 0 );    // listens for clients every hour
      Serial.println("Alarm Set for " + String(((((tm.Minute + 10) / 10) * 10) % 60) + 1));*/
      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 1, 0, 0 );
      Serial.println("Alarm Set for " + String((tm.Hour + 1)%24) +":01");
    }
    else {
      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 31, 0, 0 );   // if < 31, set alarm for :31
      Serial.println("Alarm Set for " + String((tm.Hour)%24) +":31");
    }

}
//--------------------------------------------------------------------------//
//------------------------------ Loop --------------------------------------//
//--------------------------------------------------------------------------//

void loop() {
  delay(100);
  sleepNow(); // Enter sleep
  if (RTC.alarm(ALARM_1)) {   // if Alarm 1 interrupt triggered
    Serial.println("Awake");
    delay(50);
    digitalWrite(LED_PIN,HIGH);
    
    for (byte i = 0; i < numClients; i++) {     // ping each client for data iteratively
      RTC.read(tm);
      int result = sendDataRequest(i);
      if (!result) {
        Serial.println("Failed to Communicate with Radio ID \"" + String(radioIDs[i]) +"\"!");
        delay(20);
      }
      else {
        Serial.println("Successfully Communicated with Radio ID \"" + String(radioIDs[i]) +"\"!");
        delay(20);
      }
    }
    
    if (tm.Hour >= 19 || tm.Hour < 7) {            // set alarm as appropriate (7am, :01, or :31)
      RTC.setAlarm(ALM1_MATCH_HOURS, 0, 1, 7, 0);
      Serial.println("Alarm Set for 7:01a");
      saveAllDataSD(true);     
    }
    else {
/*      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, byte((((tm.Minute + 10) / 10) * 10) % 60) + 1, 0, 0 );    // alarm for next hour
      Serial.println("Alarm Set for " + String(((((tm.Minute + 10) / 10) * 10) % 60) + 1));*/
      RTC.read(tm);
      if (tm.Minute >= 31){
      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 1, 0, 0 );
      Serial.println("Alarm Set for " + String((tm.Hour + 1)%24) +":01");
      } else {
        RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 31, 0, 0 );
        Serial.println("Alarm Set for " + String((tm.Hour)%24) +":31");
      }
    }
    digitalWrite(LED_PIN,LOW);
  }
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

//------------------Listen for Command from Server--------------------//

bool sendDataRequest(byte clientIndex) {
  /*if (!manager.init())
    Serial.println("radio failed");
    
  manager.setThisAddress(serverID);*/
//  manager.setTimeout(TIMEOUT);
//  manager.setRetries(RETRIES);
  
  uint8_t clientID = radioIDs[clientIndex];
  uint8_t data[2];
  uint8_t len = 2;
  uint8_t from;
  Serial.print("Pinging client ");
  Serial.println(String(clientID));

  RTC.read(tm);

  data[0] = tm.Minute;
  data[1] = tm.Second;
 
  //In theory, should wait retry*timeout seconds
    
  manager.sendtoWait(data, len, clientID);    // send minute and seconds values to ping client
  
//  delay(TIMEOUT/2); // COMMENTED 7/18 8:17
  uint32_t startTime = millis();
  
  while(millis() < startTime + 20000) {
//    Serial.println(String(millis() - startTime));
    if (manager.available()) {                // if response received
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      uint8_t from;
      Serial.println("Waiting for Data");
//      delay(TIMEOUT);       // COMMENTED 7/18 8:17

      while (true){ 
//        memset(buf, 0, sizeof(buf));
//        delay(100);
        if (manager.recvfromAckTimeout(buf, &len, TIMEOUT*RETRIES, &from)) {  // read in data
          if (from != clientID) {
            Serial.println("Data received from wrong client!");
            return false;
          }
          Serial.println("Data Successfully Received");
          delay(50);
          if (buf[5] == '#') {
            Serial.println("Done with Client");
            //uint8_t data[1];
            //data[0] = '\n';
            //saveData(data, clientIndex);
            return true;
          }
          Serial.println("Data Received");

          Serial.println("Length: " + String(len));
          saveData(buf, clientIndex, len);      // save data to Flash
        }
      
       
     else {
          Serial.println("No Response From Client");
          return false;
        }
    }
  }
  }
        Serial.println("Done listening for Client");
  
  /*else{
    Serial.println("sendtoWait Failed");
  }*/
  return false;
  
}


//------------------ Save Data to Flash -----------------------

void saveData(uint8_t data[RH_RF95_MAX_MESSAGE_LEN], byte clientIndex) {
  const uint32_t firstAddr = uint32_t(4096) * uint32_t(10) * uint32_t(clientIndex) + uint32_t(4096);                      // first address in portion of memory assigned to this client
  const uint32_t lastAddr = uint32_t(4096) * uint32_t(10) * uint32_t(clientIndex + 1) - uint32_t(1) + uint32_t(4096);     // last address in portion of memory assigned to this client
  const uint32_t currAddrPosition = 8 * clientIndex + 1;                 // memory location for storing current address to write to
  const uint32_t lastSavedPosition = 8 * clientIndex + 5;                // memory location for storing last address saved
  uint32_t currAddr = recallAddressAtFlashPos(currAddrPosition);         // current Flash memory address to write to
  uint32_t lastSavedAddr = recallAddressAtFlashPos(lastSavedPosition);   // current Flash memory address of last byte written
  
  if (flash.readByte(currAddrPosition + 3) == 255 && flash.readByte(lastSavedPosition + 3) == 255) {    // if allotted portion full, restart from beginning of portion
    delay(10);
    currAddr = firstAddr + 3;
    lastSavedAddr = firstAddr + 2;
    saveAddressAtFlashPos(lastSavedAddr, lastSavedPosition);
  }

  byte i = 4;
  while ((data[i] >= 32 && data[i] <= 126) || data[i] == 10 || data[i] == 13) {     // checks for valid ASCII characters
    flash.writeByte(currAddr, data[i]);       // save data byte by byte
    Serial.print(char(data[i]));
    delay(10);
    currAddr++;
    i++;
    if (currAddr > lastAddr)    // if allotted portion full, save data to SD card, erase portion and reset counters
      if (saveToSD(clientIndex, lastSavedAddr + 1, lastAddr)){
        clearFlash(clientIndex);
        lastSavedAddr = firstAddr + 2;
        saveAddressAtFlashPos(lastSavedAddr, lastSavedPosition);
        currAddr = firstAddr + 3;
      }
      else {
        return;
      }
  }
  //flash.writeByte(currAddr++, '\n');
  Serial.println();
  saveAddressAtFlashPos(currAddr, currAddrPosition);    // save currAddr value 
}

//------------------------ Save Data w/ length (same as above function) -------------------------------//

void saveData(uint8_t data[RH_RF95_MAX_MESSAGE_LEN], byte clientIndex, byte len) {
  const uint32_t firstAddr = uint32_t(4096) * uint32_t(10) * uint32_t(clientIndex) + uint32_t(4096);
  const uint32_t lastAddr = uint32_t(4096) * uint32_t(10) * uint32_t(clientIndex + 1) - uint32_t(1) + uint32_t(4096);
  const uint32_t currAddrPosition = 8 * clientIndex + 1;
  const uint32_t lastSavedPosition = 8 * clientIndex + 5;
  uint32_t currAddr = recallAddressAtFlashPos(currAddrPosition);
  uint32_t lastSavedAddr = recallAddressAtFlashPos(lastSavedPosition);
  
  if (flash.readByte(currAddrPosition + 3) == 255 && flash.readByte(lastSavedPosition + 3) == 255) {    // if allotted portion full, restart from beginning of portion
    delay(20);
    currAddr = firstAddr + 3;
    lastSavedAddr = firstAddr + 2;
    saveAddressAtFlashPos(lastSavedAddr, lastSavedPosition);
  }

  byte i = 4;
  while (i < len && (data[i] >= 32 && data[i] <= 126) || data[i] == 10 || data[i] == 13) {
    flash.writeByte(currAddr, data[i]);
    //delay(10);
    Serial.print(char(data[i]));
    delay(10);
    currAddr++;
    i++;;
    if (currAddr > lastAddr)
      if (saveToSD(clientIndex, lastSavedAddr + 1, lastAddr)){
        clearFlash(clientIndex);
        lastSavedAddr = firstAddr + 2;
        saveAddressAtFlashPos(lastSavedAddr, lastSavedPosition);
        currAddr = firstAddr + 3;
      }
      else {
        return;
      }
  }
  //flash.writeByte(currAddr++, '\n');
  Serial.println();
  saveAddressAtFlashPos(currAddr, currAddrPosition);
}

//--------------------- Save data stored in Flash to microSD card --------------------------

bool saveAllDataSD(bool erase) {
  for (byte clientIndex = 0; clientIndex < numClients; clientIndex++) {
    Serial.println(clientIndex);
    delay(100);
    const uint32_t firstAddr = uint32_t(4096) * uint32_t(10) * uint32_t(clientIndex) + uint32_t(4096);
    const uint32_t currAddrPosition = 8 * clientIndex + 1;
    const uint32_t lastSavedPosition = 8 * clientIndex + 5;
    uint32_t currAddr = recallAddressAtFlashPos(currAddrPosition);
    uint32_t lastSavedAddr = recallAddressAtFlashPos(lastSavedPosition);
    if (flash.readByte(currAddrPosition + 3) == 255 || flash.readByte(lastSavedPosition + 3) == 255) {
      delay(10);
      currAddr = firstAddr + 3;//(uint32_t)(3);
      lastSavedAddr = firstAddr + 2;
      saveAddressAtFlashPos(lastSavedAddr, lastSavedPosition);
      saveAddressAtFlashPos(currAddr, currAddrPosition);
    }
    Serial.println(clientIndex);
    delay(100);
    
    if (saveToSD(clientIndex, lastSavedAddr + 1, currAddr - 1)){     // if data transfers successfully, erase from flash and update address counters
      lastSavedAddr = currAddr - 1;
      saveAddressAtFlashPos(lastSavedAddr, lastSavedPosition);
    }
    else {
      return false;
    }

    if (erase) {
      currAddr = firstAddr + (uint32_t)(3);
      saveAddressAtFlashPos(currAddr, currAddrPosition);
      delay(100);
      saveAddressAtFlashPos(currAddr-1, lastSavedPosition);
      clearFlash(clientIndex);
    }
  }
  return true;
}

//---------------------------------------------------------------------------------------------
bool saveToSD(byte clientIndex, uint32_t startAddr, uint32_t endAddr) {
  Serial.println(String(clientIndex) + " : " + String(startAddr) + " : " + String(endAddr));
  String filename = String(clientIDs[clientIndex]) + ".csv";    // compile filename from nodeIDs
  char fn[sizeof(filename) + 4];
  filename.toCharArray(fn, sizeof(filename));
  if (fn[5] != '.') {
    fn[5] = '.';
    fn[6] = 'c';
    fn[7] = 's';
    fn[8] = 'v';
    fn[9] = '\0';
  }
  //Serial.println(sizeof(fn));
  delay(50);
  delay(50);
  sd = SDHandler(fn);                           // create file
  File file = SD.open(filename, FILE_WRITE);
  if (file && SD.exists("/TEST")) {
    file.close();                               // close file
    sd.checkHeader(fn);                         // check if header info printed into document
    file = SD.open(filename, FILE_WRITE);       // open file
    for (uint32_t i = startAddr; i <= endAddr; i++) {   // write data into file
      file.write((char)(flash.readByte(i)));
      delay(10);
    }
    file.close();                               // close to save changes
    delay(200);
  }
  else {
    Serial.println("Open file failed.");
    Serial.println(filename);
    Serial.println(clientIndex);
    //file.close();
//    digitalWrite(LED_PIN,HIGH);
    return false;
  }
  
  
  return true;
}

//----------- Erase Flash sectors allotted for data storage (all except first sector -----------------//
void clearFlash(byte clientIndex) {     
  const uint32_t firstSector = (uint32_t(10) * uint32_t(clientIndex) + uint32_t(1)) * uint32_t(4096);
  for (byte i = 0; i < 10; i++) {
    //Serial.println(firstSector + (i * 4096));
    flash.eraseSector(firstSector + (i * 4096));
    delay(50);
  }
}

//------------------ Print data saved to Flash to Serial Monitor ----------//
void printFlashData(byte clientIndex) {
  delay(100);
  const uint32_t firstAddr = uint32_t(4096) * uint32_t(10) * uint32_t(clientIndex) + uint32_t(4096);
  const uint32_t lastAddr = uint32_t(4096) * uint32_t(10) * uint32_t(clientIndex + 1) - uint32_t(1) + uint32_t(4096);
  const uint32_t currAddrPosition = 8 * clientIndex + 1;
  const uint32_t lastSavedPosition = 8 * clientIndex + 5;
  uint32_t currAddr = recallAddressAtFlashPos(currAddrPosition);
  uint32_t lastSavedAddr = recallAddressAtFlashPos(lastSavedPosition);
//  Serial.println(firstAddr);
//  delay(100);
//  Serial.println(currAddrPosition);
//  delay(100);
//  Serial.println(currAddr);
//  delay(100);
//  Serial.println(lastSavedAddr);
//  delay(100);
  
  for (uint32_t addr = firstAddr + 3; addr <= lastAddr; addr++) {
    byte b = flash.readByte(addr);
    if (b == 255)
      return;
    Serial.print(char(b));
    delay(10);
  }
}

//-----------------------Sync Client---------------------------//
void syncClient() {
  if (!manager.init())
    Serial.println("radio failed");
    
//  manager.setThisAddress(serverID);

  Serial.println("Waiting for Comm.");

  uint8_t buf[10];
  uint8_t len = sizeof(buf);
  uint8_t from;
  uint32_t startNow = millis();
  uint32_t timeout = 6000;
  
  while (millis() < startNow + timeout){

//  memset(buf, 0, sizeof(buf));
  if (manager.available()){
  if (manager.recvfromAckTimeout(buf, &len, TIMEOUT, &from)) {
    Serial.println("Request Received from " + String(char(buf[0])) + String(char(buf[1])) + String(char(buf[2])) + String(char(buf[3])) + String(char(buf[4])));
    if (!(buf[6] == (byte)((authKey >> 8) & 0xFF) && buf[7] == (byte)(authKey & 0xFF))) {
      Serial.println("Authentication Failed");
      return;
    }
    uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = 6;
    RTC.read(tm);
//    delay(100);
//    memset(data, 0, sizeof(data));
    delay(50);
    data[0] = tm.Month;
    data[1] = tm.Day;
    data[2] = tm.Year;
    data[3] = tm.Hour;
    data[4] = tm.Minute;
    data[5] = tm.Second;
    
    manager.sendtoWait(data, len, from);
    
    bool reg = false;
    for (byte i = 0; i < numClients; i++) {
      if (clientIDs[i][0] == buf[0] && clientIDs[i][1] == buf[1] && clientIDs[i][2] == buf[2] && clientIDs[i][4] == buf[4]) {
        reg = true;
        break;
      }
    }
    if (!reg) {
      Serial.println("Client Not Registered. Registering Now...");
      delay(100);
      clientIDs[numClients][0] = buf[0];
      clientIDs[numClients][1] = buf[1];
      clientIDs[numClients][2] = buf[2];
      clientIDs[numClients][3] = '_';
      clientIDs[numClients][4] = buf[4];
      byte hundreds = buf[0] - 48;
      byte num = 10 * (buf[1] - 48) + (buf[2] - 48);
      byte rep = buf[4] - 48;
      //radioIDs[numClients] = 32 * (hundreds - 1) + 2 * num + rep;
      radioIDs[numClients] = buf[5];
      flash.writeByte(105 + 5*numClients, hundreds + 48);
      flash.writeByte(105 + 5*numClients + 1, num / 10 + 48);
      flash.writeByte(105 + 5*numClients + 2, num % 10 + 48);
      flash.writeByte(105 + 5*numClients + 3, rep + 48);
      flash.writeByte(105 + 5*numClients + 4, buf[5]);
      numClients++;
    }
    break;
  }
  }

  }
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
/*void eraseFlash() {
  Serial.println("Erasing, Please Wait...");
  delay(100);
  for (int i = 1; i < flash.getMaxPage(); i++)
    flash.eraseSector(i, 0);
  currAddress = START_ADDR;
  lastSentAddr = START_ADDR - 1;
  saveCurrAddress();
  Serial.println("Erasing Complete");
  delay(100);
}*/

//-----------------------------------------------------------
//  erases clientIDs from flash, saving addresses
//-----------------------------------------------------------
void clearClientIDsFlash() {
  uint32_t flashAddresses[24];
  for (byte i = 0; i < MAX_CLIENTS; i++) {
    flashAddresses[2*i] = recallAddressAtFlashPos(4 * 2 * i + 1);
    flashAddresses[2*i+1] = recallAddressAtFlashPos(4 * 2 * i + 5);
  }
  flash.eraseSector(0, 0);

  for (byte i = 0; i < MAX_CLIENTS; i++) {
    flash.writeByte(4 * 2 * i + 1, flashAddresses[2*i] & 0xFF);
    flash.writeByte(4 * 2 * i + 2, (flashAddresses[2*i] >> 8) & 0xFF);
    flash.writeByte(4 * 2 * i + 3, (flashAddresses[2*i] >> 16) & 0xFF);
    flash.writeByte(4 * 2 * i + 4, (flashAddresses[2*i] >> 24) & 0xFF);

    flash.writeByte(4 * 2 * i + 5, flashAddresses[2*i+1] & 0xFF);
    flash.writeByte(4 * 2 * i + 6, (flashAddresses[2*i+1] >> 8) & 0xFF);
    flash.writeByte(4 * 2 * i + 7, (flashAddresses[2*i+1] >> 16) & 0xFF);
    flash.writeByte(4 * 2 * i + 8, (flashAddresses[2*i+1] >> 24) & 0xFF);
  }
}

//---------------- Save and Get Current Address from Flash (big endian) ----------------------------//
void saveAddressAtFlashPos(uint32_t address, uint32_t startAddr) {
  uint32_t flashAddresses[24];
  byte clientInfo[60];
  for (byte i = 0; i < MAX_CLIENTS; i++) {
    flashAddresses[2*i] = recallAddressAtFlashPos(4 * 2 * i + 1);
    flashAddresses[2*i+1] = recallAddressAtFlashPos(4 * 2 * i + 5);
  }
  for (byte i = 0; i < 60; i++)
    clientInfo[i] = flash.readByte(105 + i);
  flash.eraseSector(0, 0);

  for (byte i = 0; i < MAX_CLIENTS; i++) {
    if (4 * 2 * i + 1 == startAddr) { 
      flash.writeByte(startAddr, address & 0xFF);
      flash.writeByte(startAddr + 1, (address >> 8) & 0xFF);
      flash.writeByte(startAddr + 2, (address >> 16) & 0xFF);
      flash.writeByte(startAddr + 3, (address >> 24) & 0xFF);
    }
    else {
      flash.writeByte(4 * 2 * i + 1, flashAddresses[2*i] & 0xFF);
      flash.writeByte(4 * 2 * i + 2, (flashAddresses[2*i] >> 8) & 0xFF);
      flash.writeByte(4 * 2 * i + 3, (flashAddresses[2*i] >> 16) & 0xFF);
      flash.writeByte(4 * 2 * i + 4, (flashAddresses[2*i] >> 24) & 0xFF);
    }

    if (4 * 2 * i + 5 == startAddr) {
      flash.writeByte(startAddr, address & 0xFF);
      flash.writeByte(startAddr + 1, (address >> 8) & 0xFF);
      flash.writeByte(startAddr + 2, (address >> 16) & 0xFF);
      flash.writeByte(startAddr + 3, (address >> 24) & 0xFF);
    }
    else {
      flash.writeByte(4 * 2 * i + 5, flashAddresses[2*i+1] & 0xFF);
      flash.writeByte(4 * 2 * i + 6, (flashAddresses[2*i+1] >> 8) & 0xFF);
      flash.writeByte(4 * 2 * i + 7, (flashAddresses[2*i+1] >> 16) & 0xFF);
      flash.writeByte(4 * 2 * i + 8, (flashAddresses[2*i+1] >> 24) & 0xFF);
    }
  }
  for (byte i = 0; i < 60; i++)
    flash.writeByte(105 + i, clientInfo[i]);
}

uint32_t recallAddressAtFlashPos(uint32_t startAddr) {
  uint32_t address = 0;
  byte buf[4];
  buf[3] = flash.readByte(startAddr);
  buf[2] = flash.readByte(startAddr + 1);
  buf[1] = flash.readByte(startAddr + 2);
  buf[0] = flash.readByte(startAddr + 3);
  address = ((((uint32_t)buf[3]) << 0) & 0xFF) + ((((uint32_t)buf[2]) << 8) & 0xFFFF) + ((((uint32_t)buf[1]) << 16) & 0xFFFFFF) + ((((uint32_t)buf[0]) << 24) & 0xFFFFFFFF);
  return address;
}

void recallClientInfo() {
  for (byte i = 0; i < MAX_CLIENTS; i++) {
    clientIDs[i][0] = flash.readByte(105 + 5*i);
    clientIDs[i][1] = flash.readByte(105 + 5*i + 1);
    clientIDs[i][2] = flash.readByte(105 + 5*i + 2);
    clientIDs[i][3] = '_';
    clientIDs[i][4] = flash.readByte(105 + 5*i + 3);
    radioIDs[i] = flash.readByte(105 + 5*i + 4);
    //radioIDs[i] = 32 * ((clientIDs[i][0] - 48) - 1) + 2 * (((clientIDs[i][1] - 48) * 10) + (clientIDs[i][2] - 48)) + (clientIDs[i][4] - 48);
  }
}

//**************************************************************************************************************************************************************//
//**************************************************************************************************************************************************************//
//**************************************************************************************************************************************************************//
//**************************************************************************************************************************************************************//
//**************************************************************************************************************************************************************//

//-------------------- Menu --------------------------------------
void menu()
{
  if (Serial.available() > 0)
    Serial.read();

  recallClientInfo();

  //set radio ID
  serverID = EEPROM.read(EEPROM_SERVER_ID);
  manager.setThisAddress(serverID); 
  
  Serial.println(F("IRT Server"));                       // print out board info
  Serial.print(F("Server ID: "));
  Serial.println(serverID);

  Serial.println();
  
  Serial.println(F("Current Clients: "));                  // print out board info
  numClients = 0;
  for (byte i = 0; i < MAX_CLIENTS; i++) {

    Serial.print("\t");
    if (byte(clientIDs[i][0]) != 255) {
      numClients++;
      Serial.print(clientIDs[i]);
      Serial.print(" : ");
      Serial.println(radioIDs[i]);
    }
    else
      break;
    delay(50);
  }
  Serial.println();
  Serial.println(numClients);
  Serial.println();

  RTC.read(tm);
  Serial.print(F("Current date:  "));
  Serial.print(tm.Month);                               // date
  Serial.print('-');
  Serial.print(tm.Day);
  Serial.print('-');
  Serial.println(tm.Year + 1970);

  Serial.print(F("Current time:  "));
  Serial.print(tm.Hour);                                // time
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

  Serial.println();
  Serial.println(F("Menu options: "));
  
  Serial.println(F("   s  <--  Enter Server ID"));
  Serial.println(F("   i  <--  Enter Client Plot, Rep Numbers, and Radio IDs"));
  Serial.println(F("   c  <--  Set Clock"));
  Serial.println(F("   a  <--  Sync Client"));  
  Serial.println(F("   d  <--  Download Data to SD"));
  Serial.println(F("   f  <--  See list of saved files"));  
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
  byte client = 0;
  byte i = 0;
  //Serial.println(menuinput);
  switch (menuinput) {
    
    case 97:  // -------------- a --------------------
      Serial.println("Attempting to Sync With Client");
      delay(50);
      syncClient();
      menu();
      break;
    
    case 99:  // -------------- c --------------------

      Serial.println(F("Set clock: "));
      //digitalWrite(IRT_PWR,HIGH);

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

    case 100:  // -------------- d --------------------
      Serial.println(F("Saving Data to SD..."));      // download all data to SD card
      delay(100);

      //Change to save to all files
      if (saveAllDataSD(false))
        Serial.println(F("Done Saving"));
      else
        Serial.println(F("Did Not Save"));
      delay(100);
      menu();
      break;

    case 102:          // ------ f - See list of saved files ---------------------------------------------
          Serial.println(F("Saved files: "));     // list all files on SD card
          delay(10);
    
          if (!SD.begin(SD_CS)) {}
    
          root = SD.open("/");
    
          printDirectory(root, 0);
          Serial.println();

          delay(1000);
    
          menu();
          break;

    case 101:  // -------------- e --------------------
      Serial.println(F("  Are you sure you wish to download all unsaved data and erase all flash memory? (y/n): "));
      delay(100);
      eraseInput = getCharInput(timeout);
      if (eraseInput == 121) {
        if (saveAllDataSD(true))
          Serial.println(F("Done Erasing"));
        else
          Serial.println(F("Did Not Erase"));
      }
      menu();
      break;

    case 105:  // -------------- i --------------------
      plot = 0;
      clearClientIDsFlash();
      while (true) {
        Serial.println(F("Enter Plot Number, Rep, and Radio ID (Enter -1 to finish):"));
        Serial.flush();
  
        Serial.print(F(" Plot:     "));               // get Plot number
        Serial.flush();                               // decode user input
        plot = getInput(timeout);
        if (plot == -1) {
          Serial.println();
          break;
        }
        numClients++;
        Serial.print(F(" Rep:  "));                   // get radioID
        Serial.flush();
        rep = getInput(timeout);

        Serial.print(F(" RadioID:  "));                   // get radioID
        Serial.flush();
        radioIDs[client] = byte(getInput(timeout));
  
        byte hundreds = plot / 100;
        byte num = plot % 100;
        byte tens = (plot % 100)/10;
        byte ones = plot % 10;
        //radioIDs[client] = 32 * (hundreds - 1) + 2 * num + rep;
        flash.writeByte(105 + 5*client, hundreds + 48);
        flash.writeByte(105 + 5*client + 1, tens + 48);
        flash.writeByte(105 + 5*client + 2, ones + 48);
        flash.writeByte(105 + 5*client + 3, rep + 48);
        flash.writeByte(105 + 5*client + 4, radioIDs[client]);
  
        clientIDs[client][0] = hundreds + 48;
        clientIDs[client][1] = tens + 48;
        clientIDs[client][2] = ones + 48;
        clientIDs[client][3] = '_';
        clientIDs[client][4] = rep + 48;
      }

      resetFunc();//menu();                                       // go back to menu
      break;

    case 112:  // -------------- p --------------------
      for (i = 0; i < numClients; i++) {
        Serial.print("------- ");
        Serial.print(clientIDs[i]);
        Serial.println(" -------");
        printFlashData(i);
        Serial.println();
      }
      resetFunc();
      break;

    case 114:  // -------------- r --------------------
      Serial.println("Start Erasing");
      delay(100);
      flash.eraseChip();
      delay(100);
      Serial.println("Done Erasing");
      delay(100);
      resetFunc();
      break;

    case 115:  // -------------- s --------------------
      Serial.println(F("Enter Server ID:"));
      Serial.flush();
      
      serverID = getInput(timeout);
      manager.setThisAddress(serverID);
      EEPROM.write(EEPROM_SERVER_ID, serverID);                           // store settings to EEPROM
      delay(20);

      resetFunc();//menu();                                       // go back to menu
      break;

    case 120:  // -------------- x --------------------
      Serial.println(F("Exit"));                           // exit, turn off Bluetooth
      Serial.println();
      delay(10);

      break;                                               // exit switch(case) routine

    case 122: // --------------- z --------------------
      uint8_t data[1];
      data[0] = '|';
      saveData(data, 0);
      resetFunc();
      break;

    default:  // -------------- default --------------------
      Serial.println(F("Exit"));                           // exit, turn off Bluetooth
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
  boolean negative = false;

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
        if (incoming[i] == '-')
          negative = true;
        else if (incoming[i] != 13 && incoming[i] != 10) {   // ignore CR or LF
          int input = incoming[i] - 48;                     // convert ASCII value to decimal
          indata = indata * 10 + input;                 // assemble to get total value
        }
      }
      break;                                            // exit before timeout
    }
  }
  Serial.println(indata);
  if (negative)
    return -1*indata;
  else
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

//--------------- Print SD Card Directory ---------------------------------------------------------------------------

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
