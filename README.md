# RadioUno

## Changes on 12/31/2018:

> When in CHANNEL mode, pressing the switch on the rotary encoder increments 
the channel number by 10. Rolls over at 100 back to 1.

> Removed 2nd TX Key line. Replaced it with a 3rd button.

> This button has 2 functions: 
Long Press: start scan. 

> Short Press: Send CQ message using your call sign (entered in source code). (in CW mode)
Start WSPR transmission (in WSPR mode). Press again to abort.
Start CW Beacon (in BECN mode). Press again to abort.
No effect in LSB/USB.

Removed Rotary switch from starting wspr/beacon modes. Replaced with button 3 (scq switch)

> Removed scan function from mode button. Has it's own button now (see above)

> Added CQ Send routine. Short press on button 3 sends a 3x2 CQ message on the current frequency

> Added CW Beacon. Press MODE until function says BECN. Press button 3 (scq) 

A CW beacon will be sent in the following format: 
8 Seconds of carrier on the current frequency. Your callsign. Your grid square. The word
'BEACON and then the sequence begins again. It will continue forever or until 
you press button 3 for about 1/2 second. That stops the 
beacon and returns you to normal operation.

> Moved most of the constants to flash memory and wrote routines to read them.
Saved a LOT of grief with RAM space running low.

------------------------------------------------------------------


The .jpg file is the schematic of the controller as-built. This is my running 
version. It selects CW, USB/LSB (receive - I use DSB for transmit) and WSPR. 
The code is heavely documented. If you have any experience programming, you 
should have no trouble making changes to suit your setup, or just use the code as-is.

------------------------------------------------------------------

# This is an HF radio controller. 
It has the receive and transmit oscillators
using an Si5351 programmable clock oscillator, an MCP23008 port expander as a 
band register used to control the low pass filters, and using an Arduino 
ATmega328 processor. The display is a (very pretty) blue/white 16x2 LCD. Any
16x2 LCD display will work or you can change to a different size. Just change
the row/column numbers if you use a different size.

All the usual radio features are there. VFO from (settable by you) 2.9 MHz thru
15.2 MHz (changeable in #defines at the start of code), 100 memory channels, 
an SWR indicator during transmit, DC voltage display on the 2x16 char LCD, different 
modes controlling lines on the MCP23008. Tone out (during CW) with adjustable tone 
frequency (also sets CW offset in receive). Calibrate function (in menu) to set the
master oscillator frequency +/- 1 hz.

Band scan (100kc from current displayed frequency or channel frequency) is used
as a way to tell if there is any activity. 100 kc is scanned in a little over 37 seconds.
The RIT is a little different in that you can go past the usual +/- 10KC. 
Actually you can go as far apart as you like. Turning off RIT returns the receiver to 
the original frequency.

WSPR beacon on displayed frequency (either channel or vfo).

CW beacon on displayed frequency (either channel or vfo).

Single memory CW transmit (used for 'cq de callsign' in cw mode)

There are 100 pre-defined channels that can be changed during normal operation. They can
be set to default values from the menu. (Changeable in source code).

I use an Adafruit Si5351 breakout board for my clock osc.
You will need to install the Adafruit si5351 library in your arduino
library for this code to work. If you use another si5351 board (or roll your own)
you will need to make the appropriate changes. 

The code is in Beta at the moment. It's working for me, but I'm regularly adding features.
Currently the code is 25K as loaded in the Arduino.

More information in the file 'operation.txt'. Please read it.

TODO: 
> Try a different Si5351 library (limitations on the current one used)

> Clean up transmit code (not tested too well, will fix as my xmtr evolves)

> DC Voltage display does not update during transmit

>SWR display only active in CW/SSB transmit, not yet in WSPR/BECN or send CQ.






