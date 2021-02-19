
#include "SDHandler.h"

// constructor
SDHandler::SDHandler(char filename[]) {
    EEPROM.get(5, nBlocksInEEPROM);
    pinMode(LED_PIN, OUTPUT);
    //if (SD.begin(SD_CS)) {
        // create TEST directory
        SD.mkdir("/TEST");
        // open file and check header
        File file = SD.open(filename, FILE_WRITE);
        if (file && SD.exists("/TEST")) {
            //unpreserveTo(file);
            failed = false;
            digitalWrite(LED_PIN, LOW);
        } else {
            failed = true;
            Serial.println("SD initialized but open file failed. ");
            digitalWrite(LED_PIN, HIGH);
        }
        file.close();
    /*} else {
        failed = true;
        digitalWrite(LED_PIN, HIGH);
        Serial.println("SD initialization failed. ");
        flashLED();
    }*/
} // SDHandler::SDHandler

// flash LED if initialization failed
void SDHandler::flashLED() {
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 10; i++){
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
} // SDHandler::flashLED

// open file with filename and write str followed by a new line
void SDHandler::write(char filename[], int rep, float battV, tmElements_t & tm, float dataIRT[4][2]) {
    File file = SD.open(filename, FILE_WRITE);
    if (file && SD.exists("/TEST")) {
        //if (failed) unpreserveTo(file);
        file.close();
        checkHeader(filename);
        file = SD.open(filename, FILE_WRITE);
        String str = StringMaker::makeString(rep, battV, tm, dataIRT);
        file.println(str);
        failed = false;
        digitalWrite(LED_PIN, LOW);
        Serial.print("Written to SD: ");
        Serial.println(str);
    } else {
        Serial.print("Open file failed. ");
        failed = true;
        digitalWrite(LED_PIN, HIGH);
        preserve(rep, battV, tm, dataIRT);
    }
    file.close();
} // SDHandler::writeln

// write str to Flash
void SDHandler::preserve(int rep, float battV, tmElements_t & tm, float dataIRT[4][2]) {
    // put date into block
    Block block;
    block.rep = rep;
    block.battV = battV;
    block.tm = tm;
    block.copy(dataIRT);
    // put block in EEPROM
    int addr = baseAddrEEPROM + nBlocksInEEPROM * sizeof(Block);

    //prevents from overwriting EEPROM
    if (addr + sizeof(Block) <= 1023) {
      EEPROM.put(addr, block);
      // increase counter
      nBlocksInEEPROM++;
      EEPROM.put(5, nBlocksInEEPROM);
      String str = StringMaker::makeString(rep, battV, tm, dataIRT);
      Serial.print("Preserved to EEPROM: ");
      Serial.println(str);
    }
    
} // SDHandler::writeToFlash

// transfer all data from EEPROM to file
void SDHandler::unpreserveTo(File & file) {
    // transfer data
    //Serial.println("UnpreserveTo"); delay(10);
    for (int n = 0; n < nBlocksInEEPROM; n++) {
        //Serial.println("1"); delay(10);
        // get data from EEPROM
        Block block;
        int addr = baseAddrEEPROM + n * sizeof(Block);
        //Serial.println("2"); delay(10);
        EEPROM.get(addr, block);
        //Serial.println("3"); delay(10);
        // make string and write into file
        String str = StringMaker::makeString(block.rep, block.battV, block.tm, block.dataIRT);
        //Serial.println("4"); delay(10);
        file.println(str);
        //Serial.println("5"); delay(10);
        Serial.print("Unpreserve: ");
        Serial.println(str);
    }
    // reset nBlocksInEEPROM
    delay(50);
    //Serial.println("Before put");
    delay(50);
    nBlocksInEEPROM = 0;
    EEPROM.put(5, nBlocksInEEPROM);
    //Serial.println("After put");
    delay(50);
} // SDHandler::unpreserveTo

// write data output description at the begging of the file
void SDHandler::checkHeader(char filename[]) {
    File myfile = SD.open(filename, FILE_READ);
    char first = myfile.peek();
    myfile.close();
    if (first == -1) {
        // write data output description
        myfile = SD.open(filename, FILE_WRITE);
        myfile.println(F("Rep,Battery Voltage(V),Timestamp,Ground Object T (deg C),Ground Ambient T (deg C),10 deg Leaf T (deg C),10 deg Ambient T (deg C),35 deg Leaf T (deg C),35 deg Ambient T (deg C),Canopy T (deg C),Ambient Air (deg C)"));
        myfile.close();
    }
} // SDHandler::checkHeader
