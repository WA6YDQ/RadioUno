# RadioControlOsc
Arduino controller for portable HF radio with Si5351 oscillator, WSPR, multiple channels, etc

Note: This code is pre-alpha. It uses an arduino 328 and controls an Si5351 to generate
2 clock outputs: clock 0 is TX, clock 1 is 4x the receive frequency for a tayloe 
detector.
As of 12/1/2018 I made a major rewrite to this to support new hardware. As such, the code
has changes that aren't tested (and may not compile) since the new hardware is being shipped 
and can't be tested for a few days from today.

This arduino code runs a QRP-Labs tayloe based HF receiver. It uses an Si5351 for an 
oscillator, a LCD for the display, a couple of buttons for mem/vfo (and store/recall)
and mode (LSB/USB/CW-L,/CW-U) (and menu functions).

It supports multiple channels (currently 100) and an MCP23008 port expander to drive
multiple low pass filters. As the frequency changes, the port expander selects different
filters.

It also generates WSPR symbols and has been tested successfully over the air as a WSPR
transmitter.

This is on-going. 
As the hardware matures I'll post schematics. 
