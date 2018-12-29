# RadioUno

An Arduino controller for portable HF radio with Si5351 oscillator, WSPR, 
multiple channels, band scan, etc.

The .jpg file is the schematic of the controller as-built. This is my running 
version. It selects CW, USB/LSB (receive - I use DSB for transmit) and WSPR. 
The code is heavely documented. If you have any experience programming, you 
should have no trouble making changes to suit your setup, or just use the code as-is.

This is an HF radio controller. It has the receive and transmit oscillators
using an Si5351 programmable clock oscillator, an MCP23008 port expander as a 
band register used to control the low pass filters, and using an Arduino 
ATmega328 processor. The display is a (very pretty) blue/white 16x2 LCD. Any
16x2 LCD display will work or you can change to a different size. Just change
the row/column numbers if you use a different size.

All the usual radio features are there. VFO from (settable by you) 2.9 MHz thru
15.2 MHz (changeable in #defines at the start of code), 100 memory channels, 
an SWR indicator during transmit, DC voltage display on the 2x16 char LCD, different 
modes controlling lines on the MCP23008, and a WSPR subsection that lets you 
transmit WSPR from one of the channels or the VFO. 

Band scan (100kc from current displayed frequency or channel frequency) is used
as a way to tell if there is any activity. 100 kc is scanned in a little over 37 seconds.
The RIT is a little different in that you can go past the usual +/- 10KC. 
Actually you can go as far apart as you like. Turning off RIT returns the receiver to 
the original frequency.

I use an Adafruit Si5351 breakout board for my clock osc.
You will need to install the Adafruit si5351 library in your arduino
library for this code to work. If you use another si5351 board (or roll your own)
you will need to make the appropriate changes. 

The code is in Beta at the moment. It's working for me, but I'm adding features
almost daily. Currently the code is 25K as loaded in the Arduino.

TODO: 
> Try a different Si5351 library (limitations on the current one used)

> Clean up transmit code (not tested too well, will fix as my xmtr evolves)

> DC Voltage display does not update during transmit






