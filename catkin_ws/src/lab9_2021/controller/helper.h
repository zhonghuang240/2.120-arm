// Zack Bright        - zbright  _ mit _ edu,    Sept 2015
// Daniel J. Gonzalez - dgonz    _ mit _ edu,    Sept 2015
// Fangzhou Xia       - xiafz    _ mit _ edu,    Sept 2015
// Peter KT Yu        - peterkty _ mit _ edu,    Sept 2016
// Ryan Fish          - fishr    _ mit _ edu,    Sept 2016

#ifndef ArduinoLab2Helper_h
#define ArduinoLab2Helper_h

#include "Arduino.h"



//Some useful constant definitions
const float FREQ = 1000.0;                         // (Hz)
const float PERIOD = 1.0 / FREQ;
const float PERIOD_MICROS = PERIOD * 1e6;
const float SERIAL_FREQ = 100.0;                   // (Hz)
const float SERIAL_PERIOD = 1.0 / SERIAL_FREQ;
const float SERIAL_PERIOD_MICROS = SERIAL_PERIOD * 1e6;
const float b = 0.225; // (m)
const float r = 0.037; // wheel radius (m)

class SerialComm {
  public:
    float y, x;
  
    SerialComm(): y(0), x(0){
        prevSerialTime = micros();
    }
    void receiveSerialData(){
        if (Serial.available() > 0) {
            String commandString = Serial.readStringUntil('\n');  // read a line
            float command[2];
            for (int i = 0, indexPointer = 0; indexPointer != -1 ; i++ ) {
                indexPointer = commandString.indexOf(',');
                String tempString = commandString.substring(0, indexPointer);
                command[i] = tempString.toFloat();
                commandString = commandString.substring(indexPointer+1);
            }
            y = command[1];
            x = command[0];
        }
    }
  private: 
    unsigned long prevSerialTime;
};


#endif
