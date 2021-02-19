
#ifndef STRINGMAKER_H
#define STRINGMAKER_H

class StringMaker {
    const static char comma = ',';
    const static char slash = '/';
    const static char colon = ':';
    const static char space = ' ';

public:
    static String makeString(int rep, float battV, tmElements_t & tm, float dataIRT[3][2]) {
        String str = String(rep);
        makeBatteryVoltageString(str, battV);
        makeTimeString(str, tm);
        makeIRTString(str, dataIRT);
        return str;
    }

private:
    static void makeBatteryVoltageString(String & str, float battV) {
        str.concat(comma);
        str.concat(battV);
    }

    static void makeTimeString(String & str, tmElements_t & tm) {
        // assemble date
        str.concat(comma);
        str.concat(tm.Month);
        str.concat(slash);
        str.concat(tm.Day);
        str.concat(slash);
        str.concat(tm.Year + 1970);
        // assemble time
        str.concat(space);
        str.concat(tm.Hour);
        str.concat(colon);
        str.concat(tm.Minute);
        str.concat(colon);
        str.concat(tm.Second);
    }

    static void makeIRTString(String & str, float dataIRT[3][2]) {
        // numIRT -> sensor: 0 -> 90deg, 1 -> 35deg, 2 -> 10deg, 3 -> x deg
        // print: , Ground object, Ground Ambient, 10 deg Leaf, 10 deg Ambient, 35 deg Leaf, 35 deg Ambient, Canopy, Ambient Air
        for (int numIRT = 2; numIRT >= 0; numIRT--) {
            str.concat( comma );
            str.concat( dataIRT[numIRT][0] );
            str.concat( comma );
            str.concat( dataIRT[numIRT][1] );
        }
    }

};

#endif
