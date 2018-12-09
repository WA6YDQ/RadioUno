# RadioControlOsc
Arduino controller for portable HF radio with Si5351 oscillator, WSPR, multiple channels, etc

This is an HF radio controller. It has the receive and transmit oscillators
using an Si5351 programmable clock oscillator, an MCP23008 as a band register
used to control the low pass filters, and using an Arduino ATmega328 processor.

All the usual radio features are there. VFO from (settable by you) 2.9 MHz thru
15.2 MHz (changeable in #defines at the start of code), 100 memory channels, 
an SWR indicator during transmit, DC voltage display on the 2x16 char LCD, different 
modes controlling lines on the MCP23008, and a WSPR subsection that lets you 
transmit WSPR from one of the channels or the VFO.

I use an Adafruit Si5351 breakout board for my clock osc.
You will need to install the Adafruit si5351 library in your arduino
library for this code to work. If you use another si5351 board (or roll your own)
you will need to make the approiate changes. 

The code is in Beta at the moment. It's working for me, but I'm adding features.

TODO: 
add a keyer (currently cw manual only right now);
add cw beacon mode;
try a different Si5351 library (limitations on the current one used);



