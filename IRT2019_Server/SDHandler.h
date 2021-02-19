
#ifndef SDHANDLER_H
#define SDHANDLER_H

#define SD_CS 3     // ChipSelect pin for SD card SPI
#define LED_PIN 15

#include <SD.h>
#include <EEPROM.h>
#include <DS3232RTC.h>
#include "StringMaker.h"

typedef struct {
    int rep;
    float battV;
    tmElements_t tm;
    float dataIRT[4][2];
    void copy(float incoming[4][2]) {
        for (int i = 0; i < 4; i++) {
            dataIRT[i][0] = incoming[i][0];
            dataIRT[i][1] = incoming[i][1];
        }
    }
} Block;

class SDHandler {

public:
    SDHandler() {}
    SDHandler(char filename[]);
    void flashLED();
    void write(char filename[], int rep, float battV, tmElements_t & tm, float dataIRT[4][2]);
    void checkHeader(char filename[]);

private:
    bool failed; // sd card has failed at least once
    int baseAddrEEPROM = 7; // address 1 and (2-4) are for rep and plot, 5 and 6 are for int nBlocksInEEPROM
    int nBlocksInEEPROM = 0;
    void preserve(int rep, float battV, tmElements_t & tm, float dataIRT[4][2]);
    void unpreserveTo(File & file);

};

#endif
