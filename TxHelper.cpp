/*
 * TELEXi Eurorack Module
 * (c) 2016,2017 Brendon Cassidy
 * MIT License
 */
 
#include "TxHelper.h"
#include "Arduino.h"

// i2c
#include <i2c_t3.h>

TxResponse TxHelper::Parse(size_t len){

  TxResponse response;

  int buffer[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

  // zero out the read buffer
  int counterPal = 0;
  memset(buffer, 0, sizeof(buffer));

  // read the data
  while (1 < Wire.available()) {
    if (counterPal < 8) {
      buffer[counterPal++] = Wire.read();   
    }
  }
  // get the last byte
  buffer[counterPal] = Wire.read();
  
  uint16_t temp = (uint16_t)((buffer[2] << 8) + (buffer[3]));
  int16_t temp2 = (int16_t)temp;

  response.Command = buffer[0];
  response.Output = buffer[1];
  response.Value = (int)temp2;  // rh this is bytes 2 and 3 as integer
  response.byte1 = buffer[1]; // RH added raw values for the MIDI parser
  response.byte2 = buffer[2]; // RH added raw values for the MIDI parser
  response.byte3 = buffer[3]; // RH added raw values for the MIDI parser  
  response.byte4 = buffer[4]; // RH added raw values for the MIDI parser  

//  Serial.printf("cmd: %02x byte1: %02x byte2: %02x byte3: %02x byte4: %02x\n value: %d\n", buffer[0], buffer[1], buffer[2],buffer[3],buffer[4], response.Value);


  return response;
  
}

TxIO TxHelper::DecodeIO(int io) {
  
  TxIO decoded;
  
  // turn it into 0-7 for the individual device's port
  decoded.Port = io % 8;
  
  // output mode (0-7 = normal; 8-15 = Quantized; 16-23 = Note Number)
  decoded.Mode = io >> 3;

  return decoded;
}

