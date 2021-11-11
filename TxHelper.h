/*
 * TELEXi Eurorack Module
 * (c) 2016, 2017 Brendon Cassidy
 * MIT License
 */
 
#ifndef TxHelper_h
#define TxHelper_h

// i2c
#include <i2c_t3.h>

#include "Arduino.h"

struct TxResponse {
  byte Command;
  byte Output;
  int Value;
  byte byte1; // RH added raw values for MIDI parser
  byte byte2;
  byte byte3;
  byte byte4;
};

struct TxIO {
  short Port;
  short Mode;
};

class TxHelper
{
  public:

    static TxResponse Parse(size_t len);
    static TxIO DecodeIO(int io);

  protected:
    
  private:
 

};

#endif

