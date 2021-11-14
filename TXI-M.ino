/*
 * TELEXi Eurorack Module
 * (c) 2016, 2017 Brendon Cassidy
 * MIT License
 */
 // RH - this version merges some of I2C2MIDI with TXI code for my custom expander
 // I ditched most of the I2C2MIDI code because it uses the Disting ops which Teletype sends to a different I2C address
 // this version uses the generic I2C Teletype ops for MIDI in and out
 // use IIA 104 to set the I2C address to the slave address of your TXI - 104 in this case
 // use IIB2 to send MIDI messages e.g. a note on message:
 // IIB2 X90 <note> <velocity>    - X91 would send to MIDI channel 2 
 // also supports note off, control change, pitch bend, conrol change, clock and MIDI transport messages
 // to set note duration:
 // IIB1 X70 <channel> <note duration in ms> channel 0 = set all channels, 1-16 to set individual channels
 // note that if you set note duration to 0 there will be no timed note off message - you must send the note off message in your script
 // MIDI in supports CC messages only - intended for sending parameters to Teletype using an external CC controller
 // set the MIDI channel to listen to:
 // IIB1 X78 <channel>  - channel 0 is OMNI, 1-15 for MIDI 1-16
 // read the last CC value that was sent:
 // IIB1 X7b <CC#> 
 
 
// debug flag turns on serial debugging over USB
// #define DEBUG 1

// i2c Wire library (for Teensy)
#include <i2c_t3.h>
#include <EEPROM.h>
#include <MIDI.h>

// support libraries
#include "telex.h"
#include "Quantizer.h"
#include "AnalogReader.h"
#include "TxHelper.h"


// USB MIDI
// This is for your own modifications or hacks. Don't forget to select 'Tools' -> 'USB Type' -> 'MIDI + Serial' when uploading to the Teensy.
// uncomment this to use USB MIDI over Teensy's USB port:
// #define USB_MIDI

// MIDI
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI1);

// I2C2MIDI Values init
unsigned long notes[16][8][4];    // array to store the note information: pitch, start time, duration, currently on/off
int noteCount[16];
int currentNote[16];
int noteDuration[16] = {300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300};           // default note durations
int maxNotes = 8;                 // polyphony
int numChannels = 16;
int lastChannel = 1;
int controlchannel = 0;  // channel to capture CC messages on
int control;      // control requested
byte ccvalues[128] ; // captured CC values
int led1 = 2;   // I2C activity
int led2 = 3;   // midi input
int led3 = 4;   // midi output
unsigned long lastLEDMillis1 = 0;
unsigned long lastLEDMillis2 = 0;
unsigned long lastLEDMillis3 = 0;
int animationSpeed = 100;


// i2c pullup setting
bool enablePullups = false;

// config inputs
// int configPins[] = { 2, 1, 0 }; // RH I'm going to hard code the I2C address
int configID = TI;

// logging in the loop
#define LOGINTERVAL 250 // 1000
#define LEDINTERVAL 1000

// inputs, readers and storage
// CVin 4,3,2,1 pot 1,2,3,4
//int inputs[] = { A6, A7, A8, A9, A1, A3, A0, A2 };
int inputs[] = { A0, A1, A2, A3, A9, A6, A8, A7 };  // RH my version has analogs mapped a bit differently
AnalogReader *analogReaders[8];
QuantizeResponse qresponse;
int volatile inputValue[8];
int volatile quantizedValue[8];
int volatile quantizedNote[8];

// read timer and its local variables
IntervalTimer readTimer;
int p = 0;

// quantizers and quantizer state
Quantizer *quant[8];

// i2c transmission stuff
byte buffer[4];
int targetOutput = 0;

// i2c slave transmit
byte activeInput = 0;
byte activeMode = 0;

#ifdef DEBUG
unsigned long logInterval = 0;
// led status for tx/rx
int LED = 13;
unsigned long ledInterval;
bool ledOn = false;
#endif

/*
 * Setup Function
 */
void setup() {

  pinMode(led1,OUTPUT); 
  pinMode(led2,OUTPUT); 
  pinMode(led3,OUTPUT);   

  // start up animation
  for (int i=0; i < 4; i++) {
    digitalWrite(led1,HIGH); delay(animationSpeed);
    digitalWrite(led2,HIGH); delay(animationSpeed);
    digitalWrite(led3,HIGH); delay(animationSpeed);
    digitalWrite(led1,LOW); delay(animationSpeed);
    digitalWrite(led2,LOW); delay(animationSpeed);
    digitalWrite(led3,LOW); delay(animationSpeed);
  }

  // config
  int cfg = 0; 
  /*  for (i=0; i < 3; i++){
    pinMode(configPins[i], INPUT);
    cfg += digitalRead(configPins[i]) << i;
  }
*/
 
 configID += cfg;  

  // TELEXi uses the standard Teensy analog inputs which have 13 bit usable resolution
  analogReadResolution(13);

  // debugging nonsense
#ifdef DEBUG
  // set the behind-the-scenes LED output pin
  pinMode(LED, OUTPUT);
  logInterval = millis() + LOGINTERVAL;
  Serial.begin(115200);
  // wait for debugging connection   
  while (!Serial);
  Serial.printf("ConfigID: %d\n", configID);
#endif

  // initialize the readers, input values and quantizers
  for (int i=0; i <8; i++) {
    analogReaders[i] = new AnalogReader(inputs[i], i >= 4);
    inputValue[i] = 0;
    quant[i] = new Quantizer(0);
  }

  // read the calibration data from EEPROM
  readCalibrationData();

#ifdef DEBUG
  // take a quick pause (for the calibration data to print for debugging)
  delay(1000);
#endif

  // start the read timer
  readTimer.begin(readInputs, 1000); // 500

  // enable i2c and connect the event callbacks
  Wire.begin(I2C_SLAVE, configID, I2C_PINS_18_19, enablePullups ? I2C_PULLUP_INT : I2C_PULLUP_EXT, I2C_RATE_400); // I2C_RATE_2400 // I2C_PULLUP_EXT
  Wire.onReceive(receiveEvent);  
  Wire.onRequest(requestEvent);

  // Start MIDI
  MIDI1.begin(MIDI_CHANNEL_OMNI);
  
}

/*
 * the read input timer interrupt
 * need to be careful with what we access and do here
 * this function is pushing it with the quantization and stuff
 */
void readInputs(){
  // loop through the 8 inputs and store the latest value 
  for (p=0; p < 8; p++){
    inputValue[p] = analogReaders[p]->Read();
    // handle the quantized response
    qresponse = quant[p]->Quantize(inputValue[p]);
    quantizedValue[p] = qresponse.Value;
    quantizedNote[p] = qresponse.Note;
  }
}

/*
 * a simple debugging print loop - all other actions happen in the callbacks and timers
 */
void loop() {

  if (MIDI1.read()) {                    // Is there a MIDI message incoming ?

    byte type = MIDI1.getType();
    switch (type) {
      case 0xb0:   // control change
        if ((controlchannel == 0) || (MIDI1.getChannel()) == controlchannel) { // make sure its omni (channel 0) or channel we are listening on 
          ccvalues[ MIDI1.getData1()] =MIDI1.getData2(); // save the CC value for this control
          blinkLED(2);
//    Serial.printf(" msg %02x cc %02x val %02x\n", type,MIDI1.getData1(),MIDI1.getData2());  
        }
        break;
      default:
        break;
    }
  }
  
  checkNoteDurations();       // check if there are notes to turn off
  checkLEDs();                // check if the LEDs should be turned off
  
#ifdef DEBUG
/*    // print stuff
    if (millis() >= logInterval) {
        for (int l=0; l < 8; l++)
          Serial.printf("%d=%d; ", l, inputValue[l]);
        Serial.printf("\n");
        logInterval = millis() + LOGINTERVAL;
    }
*/
    // turn off LED
    if (ledOn && millis() > ledInterval) {
      ledOn = false;
      ledInterval = 0;
    }
#endif
  
}


/*
 * receive the event from the i2c wire library
 */
void receiveEvent(size_t len) {

  blinkLED(1); // blink the front panel led
#ifdef DEBUG
  // set LED active to indicate data transfer
  digitalWrite(LED, HIGH);
  ledOn = true;
  ledInterval = millis() + LEDINTERVAL;
#endif

  // parse the response
  TxResponse response = TxHelper::Parse(len);

  // true command our setting of the input for a read?
  if (len == 1) {
    
    TxIO io = TxHelper::DecodeIO(response.Command);
    
#ifdef DEBUG
    Serial.printf("Port: %d; Mode: %d [%d]\n", io.Port, io.Mode, response.Command);
#endif
    
    // this is the single byte that sets the active input
    activeInput = io.Port;
    activeMode = io.Mode;
    
  } else {
    // act on the command
    actOnCommand(response.Command, response.Output, response.Value, response.byte1, response.byte2, response.byte3, response.byte4);
  }
  
}

/*
 * this is when the master is requesting data from an input
 * we return the int (which is cast to unsigned so the sign can survive the transit)
 */
void requestEvent() {

  // disable interrupts. get and cast the value
  uint16_t shiftReady = 0;
  switch(activeMode){
    case 1:
      noInterrupts();
      shiftReady = (uint16_t)quantizedValue[activeInput];
      interrupts();
      break;
    case 2:
      noInterrupts();
      shiftReady = (uint16_t)quantizedNote[activeInput];
      interrupts();
      break;
    case 0x7b:   // RH kludgy way of sending back midi CCs - has to be in high byte
      noInterrupts();
      shiftReady = ccvalues[control] << 8; // return the last control requested
      interrupts();
      break;      
    default:
      noInterrupts();
      shiftReady = (uint16_t)inputValue[activeInput];
      interrupts();
      break;
  }
  
#ifdef DEBUG
  Serial.printf("delivering: %d; value: %d [%d]\n", activeInput, inputValue[activeInput], shiftReady);
#endif

  // send the puppy as a pair of bytes
  Wire.write(shiftReady >> 8);
  Wire.write(shiftReady & 255);
}


/*
 * act on commands delivered over i2c
 * command list is in the shared telex.h file
 */
void actOnCommand(byte cmd, byte out, int value, byte byte1, byte byte2, byte byte3,byte byte4){  // RH added the extra bytes in the I2C message for MIDI parser

  byte outHelper = out;

  // act on your commands
  switch (cmd) {

    case TI_IN_SCALE:
      outHelper += 4;
    case TI_PARAM_SCALE:
      quant[outHelper]->SetScale(value);
      break;

    case TI_IN_TOP:
      outHelper += 4;
    case TI_PARAM_TOP:
      analogReaders[outHelper]->SetTop(value);
      break;

    case TI_IN_BOT:
      outHelper += 4;
    case TI_PARAM_BOT:
      analogReaders[outHelper]->SetBottom(value);
      break;

    case TI_IN_CALIB:  
      outHelper += 4;  
    case TI_PARAM_CALIB:
      analogReaders[outHelper]->Calibrate(value);
      break;    

    case TI_STORE:
      saveCalibrationData();
      break;

    case TI_RESET:
      resetCalibrationData();
      break;
  }
  

// MIDI messages are sent using generic I2C "II" ops on the Teletype
// there is pretty much a 1:1 mapping from II ops to MIDI messages
// the Teletype sends IIB2 messages as 1 byte command <2 bytes 1st param> <2 bytes 2nd param> where param is sent as 2 bytes MSB LSB
// for MIDI we just use the LSBs which are bytes 2 and 4 in the message
// Send Midi Notes and CC

  byte midicommand = cmd & 0xf0; // mask off the channel
  byte chan = cmd & 0x0f; // get the channel number
  switch (midicommand) {
    case 0x80:
      midiNoteOff(byte2, chan);
      blinkLED(3);  
      break;
    case 0x90:
      midiNoteOn(byte2, noteDuration[chan], byte4, chan);
      blinkLED(3);  
      break;
    case 0xb0:
      sendMidiCc(byte2, byte4, chan);
      blinkLED(3);  
      break;
    case 0xc0:
      sendMidiProgramChange(byte2, chan);
      blinkLED(3);  
      break; 
    case 0xe0:
      int bendvalue = (int16_t)(byte1 << 8 | byte2);    // in this case we send a 16 bit value which is limited in sendMidiPitchBend()
      sendMidiPitchBend(bendvalue, chan);
      blinkLED(3);  
      break; 
    default:
      break;
  }

  switch (cmd) { // midi realtime messages
    case 0xf8:
      // CLOCK messages have the same status for all MIDI channels 1-16
      MIDI1.sendRealTime(midi::Clock);   // !! not optimal, because this should be 24ppq
      #ifdef USB_MIDI
          usbMIDI.sendRealTime(usbMIDI.Clock);
      #endif
      blinkLED(3);  
      break;
    case 0xfa:
      MIDI1.sendRealTime(midi::Start);
      #ifdef USB_MIDI
          usbMIDI.sendRealTime(usbMIDI.Start);
      #endif
      blinkLED(3);  
      break;    
    case 0xfb:
        MIDI1.sendRealTime(midi::Continue);
        #ifdef USB_MIDI
          usbMIDI.sendRealTime(usbMIDI.Continue);
        #endif
        blinkLED(3);
      break;
    case 0xfc:
        MIDI1.sendRealTime(midi::Stop);
        #ifdef USB_MIDI
          usbMIDI.sendRealTime(usbMIDI.Stop);
        #endif
        blinkLED(3);
      break;
    case 0x70:    // command used to set note length for a channel in milliseconds- Telex command IIB1 X70 <channel> <milliseconds> if channel is 0 set all channels
      chan = byte2 & 0x1f; // 0 = set all channels, 1-16 set that specific channel
      int dur = abs((int16_t)(byte3 << 8 | byte4));    // must be positive 
      if (chan == 0 ) for (int i=0; i<16; ++i) noteDuration[i]=dur;
      else noteDuration[(chan-1) & 0xf] = dur;    // duration must be positive 
      break;
    case 0x78:    // command used to set MIDI CC capture channel- Telex command IIB1 X78 <channel#>      
      controlchannel= byte2 &0x1f;  // 0 is omni mode, 1-16 filters only that channel
      break;
    case 0x7B:    // command used to return MIDI CC value- Telex command IIB1 X7b <cc#>  
      activeMode = 0x7b; // kludge - set mode to cc response
      control=byte2 &0x7f; // save the control specified
      break;
    default:
      break;
   }
       
#ifdef DEBUG
  Serial.printf("Action: %d, Output: %d\n", cmd, outHelper);
#endif
  
}



/*
 * saves the calibration data to the Teensy's EEPROM
 * and is careful to write only what has changed
 */
void saveCalibrationData() {

  int bitPosition = 0;
  
  uint16_t uInt16t = 0;

  // Look for the TXi Tag
  // "TXi "
  if (EEPROM.read(bitPosition) != 84) EEPROM.write(bitPosition, 84);
  if (EEPROM.read(++bitPosition) != 88) EEPROM.write(bitPosition, 88);
  if (EEPROM.read(++bitPosition) != 105) EEPROM.write(bitPosition, 105);
  if (EEPROM.read(++bitPosition) != 32) EEPROM.write(bitPosition, 32);
  ++bitPosition;
  
  for (int i=0; i < 8; i++) {
    
      int cdata[3];
      analogReaders[i]->GetCalibrationData(cdata);
      bool calibrated = analogReaders[i]->GetCalibrated();
      
      if (EEPROM.read(bitPosition) != calibrated ? 1 : 0) EEPROM.write(bitPosition, calibrated ? 1 : 0);
      ++bitPosition;
    
    for (int q=0; q< 3; q++) {
      uInt16t = (uint16_t)cdata[q];
      byte one = uInt16t & 255;
      byte two = uInt16t >> 8;
      if (EEPROM.read(bitPosition) != one) EEPROM.write(bitPosition, one);
      if (EEPROM.read(++bitPosition) != two) EEPROM.write(bitPosition, two);
      ++bitPosition;
    }
  }
    
}

/*
 * resets the calibration data to defaults
 */
void resetCalibrationData() {
  for(int i=0;i<8;i++){
    analogReaders[i]->SetCalibrationData(0, i < 4 ? 0 : -16384);
    analogReaders[i]->SetCalibrationData(1, 0);
    analogReaders[i]->SetCalibrationData(2, 16384);
  } 
}

/*
 * reads the calibration data from the Teensy's EEPROM
 */
void readCalibrationData(){
  
  int bitPosition = 0;

  uint16_t uInt16t = 0;
  
  // Look for the TXi Tag
  // "TXi "
  if (EEPROM.read(bitPosition++) == 84 && EEPROM.read(bitPosition++) == 88 && EEPROM.read(bitPosition++) == 105 && EEPROM.read(bitPosition++) == 32) {
   
    for (int i=0; i < 8; i++) {
        analogReaders[i]->SetCalibrated(EEPROM.read(bitPosition++) >= 1);
      for (int q=0; q< 3; q++) {
        uInt16t = EEPROM.read(bitPosition) + (EEPROM.read(bitPosition + 1) << 8);
        bitPosition += 2;
        analogReaders[i]->SetCalibrationData(q, (int16_t)uInt16t);
      }
    }
  
#ifdef DEBUG
  } else {
    Serial.print("skipping - eprom not initialized\n");
#endif
  }
  
}

// midi functions from I2C2MIDI

// function for sending MIDI Note On
void midiNoteOn(int pitch, int noteDuration, int velocity, int channel) {

  // check if this note is already playing; if yes, send note off message and play again
  for (int i=0; i < maxNotes; i++) {
    if (notes[channel][i][0] == pitch && notes[channel][i][3] == 1) {
      #ifdef DEBUG
        Serial.println("Note is already playing"); 
      #endif
      MIDI1.sendNoteOff(notes[channel][i][0], 0, channel+1);
      #ifdef USB_MIDI 
        usbMIDI.sendNoteOff(pitch, 0, channel+1);
      #endif
      digitalWrite(led2,LOW);
      notes[channel][i][3] = 0;
    }
  }
    
  noteCount[channel] += 1;                                  // count one note up
  currentNote[channel] = noteCount[channel] % maxNotes;     // determine the current note number
  
  // check if next note number is still playing; if yes, skip to next note number; 
  // if there's no more space available, replace the note
  for (int i=0; i < maxNotes; i++) {                        
    if (notes[channel][currentNote[channel]][3] == 1) {
      noteCount[channel] += 1; // count one note up
      currentNote[channel] = noteCount[channel] % maxNotes;
    }
    else {
      break;
    }
  }
  
  // store the values for the note in the notes array
  notes[channel][currentNote[channel]][0] = pitch;          // pitch
  notes[channel][currentNote[channel]][1] = millis();       // note start time
  notes[channel][currentNote[channel]][2] = noteDuration;   // note duration
  notes[channel][currentNote[channel]][3] = 1;              // note is on

  #ifdef DEBUG
    Serial.print(currentNote[channel]); 
    Serial.print("Note on: ");
    Serial.println(notes[channel][currentNote[channel]][0]);
  #endif
  
  MIDI1.sendNoteOn(pitch, velocity, channel+1);
  #ifdef USB_MIDI 
    usbMIDI.sendNoteOn(pitch, velocity, channel+1);
  #endif

}


// function for sending MIDI Note Off 
void midiNoteOff(int pitch, int channel) {
    MIDI1.sendNoteOff(pitch, 0, channel+1);
    #ifdef USB_MIDI 
      usbMIDI.sendNoteOff(pitch, 0, channel+1);
    #endif
    blinkLED(3);
}


// function for handling Note Offs
void checkNoteDurations() {
  unsigned long currentTime = millis();
  for (int j=0; j < numChannels; j++) {
    for (int i=0; i < maxNotes; i++) {
      if (notes[j][i][3] != 0) {
        if (currentTime - notes[j][i][1] > notes[j][i][2]) {
          if (notes[j][i][2] >0) midiNoteOff(notes[j][i][0], j);  // RH added test for zero duration which means don't send note off
          notes[j][i][3] = 0;  
        }  
      } 
    }
  }   
}


// function for sending MIDI CCs
void sendMidiCc(int controller, int value, int channel){
  MIDI1.sendControlChange(controller, value, channel+1);
  #ifdef USB_MIDI 
    usbMIDI.sendControlChange(controller, value, channel+1);
  #endif
  blinkLED(2);
}


// function for sending MIDI Program Changes
void sendMidiProgramChange(int programNumber, int channel){
  MIDI1.sendProgramChange(programNumber, channel+1);
  #ifdef USB_MIDI 
    usbMIDI.sendProgramChange(programNumber, channel+1);
  #endif
  blinkLED(2);
}


// function for sending MIDI Pitch Bend
void sendMidiPitchBend(int value, int channel){
  if (value < -8192) value = -8192; // limit to legal values
  if (value > 8191) value = 8191;
  MIDI1.sendPitchBend(value, channel+1);
  #ifdef USB_MIDI 
    usbMIDI.sendPitchBend(value, channel+1);
  #endif
  blinkLED(2);
}

// MIDI input CC message handler
void handleMIDI_CC(byte channel, byte control, byte value) {
  if (channel == controlchannel) { // save CC values only for the channel we are listening to
    ccvalues[control]=value;
  }
  blinkLED(2);
}

// function for turning on the LEDs
void blinkLED(int led) {
  if (led == 1) {
    digitalWrite(led1,HIGH);
    lastLEDMillis1 = millis();
  }
  if (led == 2) {
    digitalWrite(led2,HIGH);
    lastLEDMillis2 = millis();
  }
  if (led == 3) {
    digitalWrite(led3,HIGH);
    lastLEDMillis3 = millis();
  }
}


// function for turning off the LEDs
void checkLEDs() {
  unsigned long currentMillis = millis();
  int LEDBlinkLength = 50;
  if (currentMillis - lastLEDMillis1 >= LEDBlinkLength) {
    digitalWrite(led1,LOW);
  }
  if (currentMillis - lastLEDMillis2 >= LEDBlinkLength) {
    digitalWrite(led2,LOW);
  }
  if (currentMillis - lastLEDMillis3 >= LEDBlinkLength) {
    digitalWrite(led3,LOW);
  }
}
