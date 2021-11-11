Telex-i with MIDI in and MIDI out

This is a slightly enhanced version of the Monome Teletype input expander module. Its a mashup of Telex-i and I2C2MIDI - search the Lines forums for more details on those modules.

Hardware changes:

MIDI in and out use the same circuitry as the Motivation Radio eurorack module https://github.com/jakplugg/motivation_radio_hardware - full wave rectified input on 1/8" TRS jack so it can accept either polarity, that drives the optoisolator diode. Output is a 100 ohm resistor with clamp diodes to protect the Teensy 3.1 port in case somebody plugs in the wrong thing. MIDI in and out uses Teensy serial 1. It would be possible to use the Teensy USB port as a USB MIDI device but that is not currently implemented.

Also added activity LEDs for MIDI in and out. 

NOTE: I changed the analog pin assignments somewhat from the original Telex-I schematic which I found rather hard to follow.

This implementation is NOT script compatible with the I2C2MIDI module. Teletype expects I2C expander modules to have separate I2C addresses. The current Teensy I2C slave library does not support multiple addresses so you can't make a module that responds to Telex-I and I2C2MIDI commands at the same time - maybe possible but beyond what I want to do.

Fortunately Teletype 4.0 has generic I2C commands which are used here to implement MIDI in and out. You must first set the I2C address to the slave address of your TXI - 104 in my case:

IIA 104 

MIDI messages are pretty much a 1:1 mapping from the MIDI spec. We use the IIB2 command:

IIB2 midi_command parameter1 parameter2

e.g. this is a note on command - MIDI channel 0, note 60, veleocity 127 - X91 would send to MIDI channel 2 etc:

IIB2 X90 60 127    

Note off, control change, pitch bend, patch change, clock and MIDI transport messages are supported as well:

IIB2 midi_command parameter 

I2C2MIDI has an auto note off timer to simplify MIDI handling and that approach is extended. To set note duration:

IIB1 X70 channel duration 

channel=0  - set note duration of all channels, 1-16 to set individual channels. duration is in milliseconds

If you set note duration to 0 there will be no auto note off message - your script must generate the note off messages.

MIDI In is intended for sending parameters to Teletype from an external CC controller so this implementation supports CC messages only.

To set the MIDI channel to listen to:

IIB1 X78 channel  ; channel 0 is OMNI, 1-15 for MIDI 1-16

To read the last CC value that was sent:

IIB1 X7b CC# 

Only one CC message is stored so if you want to respond quickly to CC messages you have to poll the module frequently.


Rich Heslip rheslip@hotmail.com

