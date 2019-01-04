
/*
* --- RadioUno ---
* Multi-mode radio controller and synthesizer using an Arduino Uno
* and si5351 oscillator. Also used is an MCP23008 I/O expander
* for RF filter and hardware control of modules.
* (C) 2018,2019 Kurt Theis
*
*   This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
*
* THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
* APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT
* HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS" WITHOUT WARRANTY
* OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
* IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF
* ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
*
* 
* Please see the LICENCE file in the archive for more information. 
*
* k theis <theis.kurt@gmail.com> Initial Upload 11/2018
*
* Ver2 - has 16x2 LCD, a mode/RIT button, a channel/vfo store/recall
* button, a rotary (mechanical) encoder with a button. The button steps
* thru tuning steps from 10hz thru 100 khz depending on long or short pushes.
*
* There is a MCP23008 I2C port expander that controls relays for band filters,
* cw/ssb tx/rx control and usb/lsb relay control for the receiver.
*
* The oscillator is a Si5351 used for Tx and Rx.
*
* Note: there is no provision for a frequency lockout. This will transmit
* across all frequencies. YOU are responsible to follow all laws in your
* region regarding what frequencies (if any) you can transmit on.
*
* Please Note: You can probably save some flash memory by tightening up many
* of these routines, but that would make it harder for a beginning programmer
* to understand. As long as there is available memory, I keep it verbose.
*
* TODO: 
*       redo the osc code. Some freqs I get weird noise I suspect might be caused by jitter etc.
*       show ref power in all tx modes
*       try a different Si5351 library (or write code to turn on/off indiv Si ports)
*    
*/
            
/* Subroutines:
* 
* updateBand()   look at current frequency, send filter data to port expander
* updateFreq()   show the current frequency and channel data on the LCD display
* updateMode()   change register to one of cw-L, cw-U, lsb, ssb, update display
*                and update the port expander for hardware control
* Save()         Save frequency and mode to EEPROM based on channel #
* Recall()       Read frequency and mode from EEPROM, update registers and display
* UpdateDcVolt() Read analog pin, divide by constant for true voltage, display on LCD
* changeFreq()   Interrupt driven - read rotary encoder, change freq/channel/menu etc
* updateOsc()    Read from freq and mode. Send display freq to osc0, derived rx to osc1
* menu()         Menu sub-system. Control registers, run special functions
* setDefault()   Initialize the EEPROM with default frequencies & settings
* wspr(freq)     Transmit WSPR on (freq) Call and Grid are predefined before setup() is called.
*                Power level is defined in the wspr() routine.
* txKey()        key the transmitter (send 1 to GP5 of the MCP23008), set Tx freq to display
* txDekey()      dekey the transmitter (send 0 to GP5 of the MCP23008), shift Tx freq to 1 MHz
* scan()         Scan 100kc from current display freq (vfo or channel)
* showTune()     Place cursor on digit being tuned for step size display
*/

/*
* Notes - 
* At startup, the TX freq is set to 1 MHz. The Adafruit libraries I use for the
* Si5351 don't allow the individual clocks to be enabled/disabled. It's all
* or nothing. With the TX freq set to RX all the time, it's signal is heard on the receiver.
*
*  (osc notes):
*  Enabled desired outputs (see Register 3)
*  ASSERT_STATUS(write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, enabled ? 0x00: 0xFF));
* This should enable/disable indiv clocks
* bit: 2 is CLK2_EN  1 is CLK1_EN   0 is CLK0_EN
* located in routine: Adafruit_SI5351::enableOutputs(bool enabled)
* (the etherkit libs seem more suited for this - see github.com/etherkit/Si5351Arduino)
*
*
*/

/* Operation:
*
* At start up, the status of VFO/CHAN is checked:
*
* At start up, if the CHANNEL/VFO button is NOT pressed, the radio will act normally.
* If the CHANNEL/VFO button IS pressed, you will jump into the MENU mode. This is used 
* to configure various settings. To exit from the menu mode, turn off the radio. Turning
* the radio back on while NOT pressing the CHANNEL/VFO button will restore normal operation.
*
* If not pressed, the various registers are set up (some read from eeprom) and
* the oscillator is configured. The TX osc (and osc 2) are set to 1 MHz. This gets them
* out of the way of the receiver. The receiver oscillator is configured to 4 times the
* receive frequency. The Tayloe detector my receiver uses requires this. If you have 
* a different receiver, your requirements will be different.
*
* After all the lines and registers are configured, the program enters a command loop where
* the control buttons, the rotary encoder and tx lines are constantly checked. The display
* is not updated unless external inputs change. This eliminates a lot of digital noise.
*
* All the control (input) lines tie to the processor directly. If using the port expander for
* input, it would have to be constantly polled and this would generate too much digital noise
* for the receiver to be useful. 
*
* Many additional functions I wanted proved to be impossible to impliment due to RAM limitations.
* The stack and the heap would collide causing weird issues. If you add features be aware of this.
*
*
*
* UPDATES:
*
*  1/04/2019 Minor doc changes, added extensive licence info 
*  1/03/2019 In setting grid square, start with current char under cursor
*  1/02/2019 Change memory allocation in beacon()
*  1/02/2019 Update comments (documentation changes)
*  1/01/2019 Changed encoder sw press to scq button. scq short - beacon/cq. Long/scan
* 12/31/2018 When in channel mode, encoder button press increments channel by 10
* 12/31/2018 Put static values for cw code and wspr in PROGMEM
* 12/31/2018 Cleaned up misc compiler warnings (none critical)
* 12/31/2018 Code considered useful enough to use. Still some things to be added (none necessary)
* 12/30/2018 Removed 2nd tx keyline, made it 3rd button for scan and cq option
* 12/30/2018 Wrote cw char send routine/beacon routine/cq send routine
*/




/****************************/

#define MINFREQ  2400000    // lowest operating frequency (hz)
#define MAXFREQ 16100000    // highest operating frequency (hz)

// EEPROM addresses 0-500 used for channel storage, 5 bytes/channel
#define gridAddr 504        // eeprom storage for grid square (4 bytes) (504-507)
#define SidetoneLow 508     // sidetone/cw offset frequency storage (2 bytes) (508-509)
#define SidetoneHi  509
#define CalLow 510          // EEPROM address for calibrate function (510-511)
#define CalHi  511          // (2 bytes)


#define DEBOUNCE 50         // debounce make/break switches

/****************************/

#define CALLSIGN "D0MMY"    // Your call sign for wspr, cw, beacon. Change before use! Use UPPER CASE.
                            // grid square is set from menu and stored in eeprom
#define RFPOWER 13          // rf power used in wspr (values: 0,3,7,10,13,17,30,23,27,30,33,37,40,43)

/****************************/




#include <LiquidCrystal.h>
#include <stdlib.h>
#include <EEPROM.h>
#include <Wire.h>            // for I2C port expander
#include <Adafruit_SI5351.h> // for clock osc
#include <avr/pgmspace.h>    // probably not needed
#include <avr/io.h>          // probably not needed
#include <string.h>          // probably not needed


Adafruit_SI5351 clockgen = Adafruit_SI5351();  // you will need to install the Adafruit Si5351 libs

// lcd1 is Frequency Display (LCD D0 thru D3 are unused and left floating)
LiquidCrystal lcd1(10,13,6,7,8,9);    // RS, e, D4, D5, D6, D7
// (original version used 2 lcd displays, lcd1 and lcd2)
 
/* define input lines (output lines are on the port expander, I2C & LCD) */ 
const int knobsw = 4;      // digital pin D4 (encoder switch) White wire on mine
const int knob = 2;        // digital pin D2 (encoder pulse) Brown wire on mine
const int knobDir = 5;     // digital pin D5 (encoder direction) Blue wire on mine

const int dcVoltage = A0;  // read dc voltage (thru 10K trimmer resistor) on A0
const int toneOut = A1;    // audio tone out while in transmit on A1
const int refIn = A3;      // dc reflected power input (used in transmit)
const int vc = 3;          // pushbutton, vfo/channel and sto/rcl on D3
const int keyIn1 = 11;     // key line  (D11)
const int scq = 12;        // Scan/Send CQ (D12)
const int mr = A2;         // Mode/RIT



// used for WSPR, beacon and sending CQ message
const char call[] = CALLSIGN;       
char cwMsg[] = " \0";       // initialize cw play message


// define variables

float VOLT;                 // read DC Voltage on pin A0
float freq;                 // main frequency in Hz
float ritFreq;              // tx freq when rit is 1
byte  rit;                  // 0, no rit. 1, rit active
float freqBak;              // use this when in channel mode to hold the vfo freq
float STEP;                 // step size when tuning (in Hz)
int   FREQFLAG = 0;         // when HIGH update freq Display & Osc
int   vfoChan = 0;          // 0=vfo, 1=chan, 2=menuMode
int   chan = 0;             // channel number - 0-99
int   menu_sel;             // used in menu sub-system
float MULTI = 28.0;         // multiplier (* XTAL) for PLL (used in transmit pll only)
float XTAL = 25.0;          // Crystal frequency of PLL clock (MHz)
unsigned int SIDETONE;      // sidetone frequency for tone out and CW offset
byte  radioReg;             // set radio filters, radio mode etc (for MCP23008)
int   CALOFFSET;            // calibrate setting for +/- 0 hz

#define MAXMODE 6           // set max number of modes
byte MODE;
const char mode[MAXMODE][6] = {"LSB ","USB ","CW-U","CW-L","WSPR","BECN"};

/* the following are constants used in sendCw()   */
const PROGMEM char numCw[10][7]  = {
            "333330","133330","113330","111330","111130",   // 0-4
            "111110","311110","331110","333110","333310"    // 5-9
            }; 

const char chrCw[26][6] PROGMEM = {
            "130","31110","31310","3110","10","11310",      // a-f
            "3310","11110","110","13330","3130","13110",    // g-l
            "330","310","3330","13310","33130","1310",      // m-r
            "1110","30","1130","11130","1330","31130","31330","33110"  // s-z
            };

/* the following are constants used in wspr */
const byte sync[] PROGMEM  = { 
    1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0,
    1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1,
    0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1,
    1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1,
    0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1,
    1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0
};





/*****************************/
/* set up hardware ports etc */
/*****************************/

void setup() {
  
   /* init LCD displays */
   lcd1.begin(16,2);   // initialize LCD display
   
   /* set up input devices */
   pinMode(knobsw, INPUT_PULLUP);
   pinMode(knob, INPUT_PULLUP);
   pinMode(knobDir, INPUT_PULLUP);
   pinMode(vc, INPUT_PULLUP);
   pinMode(keyIn1, INPUT_PULLUP);
   pinMode(scq, INPUT_PULLUP);
   pinMode(mr, INPUT_PULLUP);
   analogReference(DEFAULT);    // use 5vdc for this 5v arduino
   
   /* debug code - enable when problems */
   //Serial.begin(9600);
   //Serial.print("Starting up\n\n");
   
   /* set up I2C */
   Wire.begin();
   
   /* initialize port-expander */
   Wire.beginTransmission(0x20);    // select port expander
   Wire.write(0x00);                // select IODIRA of port expander
   Wire.write(0x00);                // PORT A all output
   Wire.endTransmission();          // stop talking to port expander
   
   /* initialize Si5351 oscillator */
   if (clockgen.begin() != ERROR_NONE) {
     lcd1.home();
     lcd1.print(F("CLK ERR"));     // if it doesn't start up, no reason to continue
     while (true) continue;        // so loop until power cycled
   }
      
   /* set up PLL B for rx freq range (12-120 MHz) */
   clockgen.setupPLLInt(SI5351_PLL_B, 40);  // PLL B is 1000 MHz (noisy at 900MHz)
   /* set up PLL A for TX freq range (3-30 MHz) */
   clockgen.setupPLLInt(SI5351_PLL_A, 28); // PLL A is 700 MHz (Using 25 MHz clock on Adafruit Si5351 breakout)
   /* turn on the outputs */
   clockgen.enableOutputs(true);
   
   /* set interrupts last */
   attachInterrupt(0, changeFreq, LOW);    // set interrupt on knob going LOW
   interrupts();                           // allow interrupts (normally defaults on)
}
 
 
 
 
            /***********************/ 
            /***** SUBROUTINES *****/
            /***********************/
 
 
 
/*********************************************************************/
/**** show tune speed by displaying underline cursor in freq area ****/
/*********************************************************************/

void showTune() {
    extern float STEP;
    extern int vfoChan;
    if (vfoChan != 0) return;
    
    if (STEP == 100000) {
        lcd1.setCursor(8,0);
        lcd1.cursor();
        return;
    }
    if (STEP == 10000) {
        lcd1.setCursor(9,0);
        lcd1.cursor();
        return;
    }
    if (STEP == 1000) {
        lcd1.setCursor(10,0);
        lcd1.cursor();
        return;
    }
    if (STEP == 100) {
        lcd1.setCursor(12,0);
        lcd1.cursor();
        return;
    }
    if (STEP == 10) {
        lcd1.setCursor(13,0);
        lcd1.cursor();
        return;
    }
    return;
}
 
 
 
 
 
/********************************************/ 
/****      show the current frequency    ****/
/********************************************/

void updateFreq() {  
   extern float freq;
   extern int vfoChan;

   if (vfoChan == 1) {       // show channel number (ie CH 01 14250.50)
     lcd1.home();
     
     if (chan < 10) lcd1.print(F("CH 0"));  // thats a '0' (zero)
     if (chan > 9) lcd1.print(F("CH "));
     lcd1.print(chan);
     lcd1.print(F(" "));
     // now show the frequency
   }
   
   if (vfoChan == 0) {    // show frequency of VFO (ie VF 01 14250.00)
     lcd1.home();
     
     if (chan < 10) lcd1.print(F("VF 0"));  // thats a '0' (zero), not an O (oh)
     if (chan > 9) lcd1.print(F("VF "));
     lcd1.print(chan);
     lcd1.print(F(" "));
   }
   
   // now show the frequency 
   if ((freq/1000) < 10000) lcd1.print(F(" "));    // push display out, no leading 0
   lcd1.print(freq/1000);
   lcd1.print(F(" "));    // clean up from channel display
   
   return;
}






/********************************************/ 
/**** Store vfo frequency/MODE to EEPROM ****/
/********************************************/

void Save() {
  extern float freq;
  extern int chan;
  extern byte MODE;
  int i;
  int address = chan * 5;  // store freq as (4 byte) long, mode as 1 byte
  union u_tag
  {
      byte b[4];
      float fval;
  } u;
  u.fval = freq;
  for (i=0; i<4; i++) {
  EEPROM.write(address+i, u.b[i]);  // save float value of freq
  }
  EEPROM.write(address+4, MODE);    // save mode value

  return;
}
  
  
  
  
/****************************************/
/*** Recall from EEPROM to freq/MODE ****/
/****************************************/

void Recall() {
  extern float freq;
  extern int chan;
  extern byte MODE;
  int i;
  int address = chan * 5;  // recall freq as 4 byte values, mode as 1 byte
  union u_tag
  {
      byte b[4];
      float fval;
  } u;
  for (i=0; i<4; i++) {
    u.b[i] = EEPROM.read(address+i);
  }
  freq = u.fval;
  MODE = EEPROM.read(address+4);    // get mode value
  
  return;
} 
  
  
  
  
   
/******************************/
/**** Display D.C. Voltage ****/
/******************************/

void updateDcVolt() {

    
  // read analog pin, divide by constant for true voltage (I use a 10k multiturn pot)
  extern float VOLT;
  char buf[5];

  VOLT = analogRead(dcVoltage)/42.0;    // read DC voltage (use a float here)
  // dtostrf(float var,str len, digits after decimal point, var to hold val)
  dtostrf(VOLT,4,1,buf);  // arduino can't handle floats (WTF? it has a c compiler)
  // this stupid little routine takes 2K of memory!!
  lcd1.setCursor(11,1);
  lcd1.print(buf);
  lcd1.setCursor(15,1);
  lcd1.write("V");  // show as voltage

  return;
}
 
 
 
  
/*****************************************************/
/**** INTERRUPT 0 ( rotary encoder turned cw/ccw) ****/
/*****************************************************/

/* read dir, change freq +/- by STEP size  ****/
void changeFreq() {
   extern float freq;
   extern int FREQFLAG;
   extern int menu_sel;    // determines menu selection
   
   if (vfoChan == 3) return; // in cal/sidetone/setGrid mode, vfochan = 3
   
   if ((FREQFLAG==1) && (vfoChan<3)) {  // wait until display updated before continuing
       return;
   }
   
   delay(1);      // deal with debounce using cheap encoders
   if (digitalRead(knob) == HIGH) {
     interrupts();
     return;
   }
   
   noInterrupts();  // stop further ints while in this routine
   
   if ((digitalRead(knobDir) == HIGH) && (digitalRead(knob) == LOW)) {
     
     if (vfoChan == 2) {    // in menu sub-system
       menu_sel++;
       if (menu_sel > 4) menu_sel = 0;
       FREQFLAG = 1;  
     }
     
     if (vfoChan == 0) {  // in vfo mode - increment vfo freq
       freq += STEP;
       if (freq > MAXFREQ) freq = MAXFREQ;  // freq limits
       FREQFLAG = 1;
     }
     
     if (vfoChan == 1) {  // in channel mode - increment channel number
       chan += 1;
       if (chan > 99) chan = 0;
       Recall();  // read freq from EEPROM save in freq
       FREQFLAG = 1;
     }
     
     while (digitalRead(knob)==LOW) continue;
     interrupts();  // resume ints
     return;
   }
   
   if ((digitalRead(knobDir) == LOW) && (digitalRead(knob) == LOW)) {
     
     if (vfoChan == 2) {    // in menu sub-system
       menu_sel--;
       if (menu_sel < 0) menu_sel = 4;
       FREQFLAG = 1;
     }
     
     if (vfoChan == 0) {  // in vfo mode - decrement vfo frequency
       freq -= STEP;
       if (freq < MINFREQ) freq = MINFREQ;  // freq limits
       FREQFLAG = 1;
     }
     
     if (vfoChan == 1) {  // in channel mode - decrement channel number
       chan -= 1;
       if (chan < 0) chan = 99;
       Recall();      // get the EEPROM channel freq
       FREQFLAG = 1;
     }
     
     while (digitalRead(knob)==LOW) continue;
     interrupts();  // resume ints
     return;
   }
   
   // nothing happend (shouldn't really get here)
   interrupts();
   delay(1);
   return;    
}
   
 
 
   
/***************************************/ 
/* update the si5351 oscillator for RX */
/***************************************/

void updateOsc() {

   extern unsigned int SIDETONE;
   extern float freq, XTAL;  
   extern byte MODE;   // 0=LSB, 1=USB, 2=CW-U, 3=CW-L, 4 and up USB
   float VB, VALUE, DIV, VA, VINT;
   float rxFreq;  // rx freq = 4x freq, +/- ssb/cw offset, +/- caloffset
   extern int CALOFFSET;
   
   rxFreq = 0;    // keep compiler from bitching about uninitialized vars
   VB = 1000000.0;
   
   // set up the receive frequency (no offset for ssb needed with qrp-labs rx with poly)
   if (MODE == 0) rxFreq = freq; // LSB
   if (MODE == 1) rxFreq = freq; // USB
   if (MODE == 2) rxFreq = freq - (float)SIDETONE;    // CW-L
   if (MODE == 3) rxFreq = freq + (float)SIDETONE;    // CW-U
   if (MODE >  3) rxFreq = freq; // all else
   
   rxFreq += CALOFFSET;
   
   /* When rxFreq > MAXFREQ set rxFreq to 1 mhz. (Keep si5351 happy) */
   if (rxFreq > MAXFREQ) rxFreq = 1000000;    // channels can contain freq above MAXFREQ (ie. 6M beacon)
   
   rxFreq *= 4.0;      // using a tayloe detector, rx freq is 4X display freq
   VALUE = rxFreq/VB;
   DIV = XTAL * 40;    // MULTI for PLL B is 40 (below that osc output is jittery
   VALUE = DIV/VALUE;  // above about 118 MHz)
   VINT = (long)VALUE;
   VA = (long)((VALUE - VINT) * VB);
   clockgen.setupMultisynth(1, SI5351_PLL_B, VINT, VA, VB);  // output on osc 1
   
   return;
}




/*******************/
/* Update the Mode */
/*******************/

void updateMode() {
  extern byte radioReg;
  extern byte MODE;
/* 
    |   7  |   6   |   5  |  4  |  3  |   2   |   1  |   0  |
     cw/ssb usb/lsb    tx          (band filter settings)


cw/ssb is 0 for cw, 1 for ssb
usb/lsb is 0 for lsb, 1 for usb
tx = 0 for RX, 1 for TX
*/ 

  if (MODE == 0) {    // LSB
    radioReg &= B00000111;    // clear current mode
    radioReg |= B10000000;    // set SSB-LSB
  }
  if (MODE == 1) {   // USB
    radioReg &= B00000111;    // clear current mode
    radioReg |= B11000000;    // set SSB-USB
  }
  if (MODE == 2) {   // CW-U
    radioReg &= B00000111;    // clear current mode
    radioReg |= B01000000;    // set CW-USB
  }
  if (MODE == 3) {   // CW-L  
    radioReg &= B00000111;    // clear current mode
    radioReg |= B00000000;    // set CW-LSB  
  }
  if (MODE >= 4) {   // wspr etc use USB
    radioReg &= B00000111;    // clear current mode
    radioReg |= B11000000;    // set SSB-USB
  }

  
  Wire.beginTransmission(0x20);  // set up communication with port expander
  Wire.write(0x09);              // select GPIO pins
  Wire.write(radioReg);          // set band pins
  Wire.endTransmission();        // done
  
  /* update LCD display */
  lcd1.setCursor(0,1);           // row 1 pos 0
  lcd1.print(mode[MODE]);
  
  return;
}





/*************************************************************************/
/* update the band register for the TX low pass filters and rx filters   */
/*************************************************************************/

void updateBand() {
  extern float freq;
  extern byte radioReg;
/* 
    |   7  |   6   |   5  |  4  |  3  |   2   |   1  |   0  |
     cw/ssb usb/lsb    tx        11-15   8-11    5-8   2-5
     
2-5 is a high pass at 3 mc, low pass at 5 mc 
5-8 is a high pass at 5, low pass at 8 mc
8-11 is a high pass at 8 mc, low pass at 11 mc
11-15 is a high pass at 11 mc, low pass at 15 mc

cw/ssb is 0 for cw, 1 for ssb
usb/lsb is 0 for lsb, 1 for usb
tx = 0 for RX, 1 for TX

*/ 
  if ((freq >= 2900000) && (freq < 5000000)) { // 2.9 mc thru 5 mc
    radioReg &= B1110000; radioReg |= B00000001;  // set band 1 line
  }
  if ((freq >= 5000000) && (freq < 8000000)) {  // 5 mc thru 8 mc
    radioReg &= B11110000; radioReg |= B00000010;  // set band 2 line
  }
  if ((freq >= 8000000) && (freq < 11000000)) {  // 8 mc thru 11 mc
    radioReg &= B11110000; radioReg |= B00000100;  // set band 3 line
  }
  if ((freq >= 11000000) && (freq < 15200000)) {  // 11 mc thru 15.2 mc
    radioReg &= B11110000;  radioReg |= B00001000;  // set band 4 line
  }
  
  Wire.beginTransmission(0x20);  // set up communication with port expander
  Wire.write(0x09);              // select GPIO pins
  Wire.write(radioReg);          // set band pins
  Wire.endTransmission();        // done
  
  return;
}





/*******************************************/
/* MENU mode - set default, working values */
/*******************************************/

void menu() {
  int i;                  // gp variable
  int charPos;            // used in setGrid, setCall
  int charValue;          //   ""      ""       ""
  extern int vfoChan;
  extern int menu_sel;
  
  // this is for the cal routine
  extern int CALOFFSET;
  int ByHi, ByLo;
  extern int FREQFLAG;
  float VALUE, DIV, VINT, VA, VB;
  VB = 1000000.0;
  
  // these next 2 are for grid square calcs
  char gs[5];
  char sChar[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  
  vfoChan = 2;    // in menu mode
  menu_sel = 0;   // select menu item #
  FREQFLAG = 0;
  extern unsigned int SIDETONE;
  SIDETONE = 700;
  
  lcd1.home();
  lcd1.print(F("Menu           "));
  while (digitalRead(vc) == LOW){
     delay(100);
     continue;  // wait until vc released
  }
  delay(150);
  lcd1.home();
  lcd1.print(F("Select List        "));
  delay(500);
  FREQFLAG = 1;
  
  
  /*******************/
  /**** menu loop ****/
  /*******************/
  
  while (true) {
    
    /*****************/
    /* calibrate VFO */
    /*****************/
    
    if (menu_sel == 0) {
      if (FREQFLAG == 1) {
          lcd1.home();
          lcd1.print(F("Calibrate Osc        "));
          FREQFLAG = 0;
      }
      CALOFFSET = 0;    // offset in Hz
        if (digitalRead(vc) == LOW) {    // wait for press on vc
          while (digitalRead(vc) == LOW) continue; // wait until vc is released
          delay(DEBOUNCE);
          lcd1.clear();
          lcd1.print(F("Rotate knob"));
          lcd1.setCursor(0,1);
          lcd1.print(CALOFFSET);
          vfoChan = 3;  // disable knob operation for menu selection
          VALUE = (10000000.0 + CALOFFSET)/VB;  // 10mc + offset in hz
          DIV = XTAL * MULTI; VALUE = DIV/VALUE; VINT = (long)VALUE; VA = (long)((VALUE - VINT) * VB);
          clockgen.setupMultisynth(2, SI5351_PLL_A, VINT,VA,VB); // output on osc 2
          while (true) {  // use knob to vary CALOFFSET
            if ((digitalRead(knobDir) == HIGH) && (digitalRead(knob) == LOW)) {
              CALOFFSET += 1;
              VALUE = (10000000.0 + CALOFFSET)/VB;  // 10mc + offset in hz
              DIV = XTAL * MULTI; VALUE = DIV/VALUE; VINT = (long)VALUE; VA = (long)((VALUE - VINT) * VB);
              clockgen.setupMultisynth(2, SI5351_PLL_A, VINT,VA,VB); // output on osc 2
              lcd1.setCursor(0,1);
              lcd1.print(CALOFFSET);
              lcd1.print(F(" hz       "));
              while (digitalRead(knob) == 0) continue;
            }
            if ((digitalRead(knobDir) == LOW) && (digitalRead(knob) == LOW)) {
              CALOFFSET -= 1;
              VALUE = (10000000.0 + CALOFFSET)/VB;  // 10mc + offset in hz
              DIV = XTAL * MULTI; VALUE = DIV/VALUE; VINT = (long)VALUE; VA = (long)((VALUE - VINT) * VB);
              clockgen.setupMultisynth(2, SI5351_PLL_A, VINT,VA,VB); // output on osc 2
              lcd1.setCursor(0,1);
              lcd1.print(CALOFFSET);
              lcd1.print(F(" hz       "));
              while (digitalRead(knob) == 0) continue;
            }
            if (digitalRead(vc) == LOW) {    // vc pressed - save value to eeprom
              EEPROM.write(CalLow, highByte(CALOFFSET));
              EEPROM.write(CalHi, lowByte(CALOFFSET));
              lcd1.clear();
              lcd1.print(F("Offset Saved"));
              while(digitalRead(vc) == LOW) continue;
              delay(DEBOUNCE);
              vfoChan = 2;  // re-enable knob for menu ops
              FREQFLAG = 1; // to redraw menu choice
              break;
            }
          }
        } 
      continue;
    }



    /**********************************/
    /* set default channels in EEPROM */
    /**********************************/
    
    if (menu_sel == 1) {
        if (FREQFLAG == 1) {
            lcd1.home();
            lcd1.print(F("set defaults  "));
            FREQFLAG = 0;
        }
        if (digitalRead(vc) == 0) {
          delay(100);
          if (digitalRead(vc) == 0) {
            lcd1.home();
            lcd1.print(F("writing       "));
            while (digitalRead(vc) == LOW) continue;
            delay(DEBOUNCE);
            setDefault();
            lcd1.home();
            lcd1.print(F("done           "));
            delay(750);
            FREQFLAG = 1; // to redraw menu choice
          }
        }
      delay(DEBOUNCE);
      continue;
    }
    
    
    
    
    /*********************/
    /* set sidetone freq */
    /*********************/
    
    if (menu_sel == 2) {
      if (FREQFLAG == 1) {
          lcd1.home();
          lcd1.print(F("Sidetone      "));
          FREQFLAG = 0;
      }
      if (digitalRead(vc) == 0) {
          delay(100);
          if (digitalRead(vc) == 0) {
              ByLo = EEPROM.read(SidetoneLow);
              ByHi = EEPROM.read(SidetoneHi);
              SIDETONE = word(ByHi,ByLo);
              tone(toneOut, SIDETONE); 
              vfoChan = 3;
              lcd1.home();
              lcd1.print(F("Rotate Knob   "));
              lcd1.setCursor(0,1);
              lcd1.print(SIDETONE);
              while (digitalRead(vc) == 0) continue;
              delay(DEBOUNCE);
              while(digitalRead(vc)==1) {
                  if ((digitalRead(knobDir) == HIGH) && (digitalRead(knob) == LOW)) {
                      SIDETONE += 1;
                      if (SIDETONE >= 1000) SIDETONE = 1000;
                      while (digitalRead(knob) == LOW) continue;
                      FREQFLAG = 1;
                  }
                  if ((digitalRead(knobDir) == LOW) && (digitalRead(knob) == LOW)) {
                      SIDETONE -= 1;
                      if (SIDETONE <= 500) SIDETONE = 500;
                      while (digitalRead(knob) == LOW) continue;
                      FREQFLAG = 1;
                  }
                  if (FREQFLAG == 1) {
                      tone(toneOut, SIDETONE); 
                      lcd1.setCursor(0,1);
                      lcd1.print(SIDETONE);
                      lcd1.print(F("  "));    // remove trailing zero at 1000 hz
                      FREQFLAG = 0;
                  }
              }
              while (digitalRead(vc) == 0) continue;
              delay(DEBOUNCE);
              noTone(toneOut);
              /* now save to eeprom */
              EEPROM.write(SidetoneHi, highByte(SIDETONE));
              EEPROM.write(SidetoneLow, lowByte(SIDETONE));
              lcd1.clear();
              lcd1.print(F("Saved Value   "));
              delay(700);
              FREQFLAG = 1;    // to redraw display
          }
      }
      vfoChan = 2;  // back to menu mode
      delay(DEBOUNCE);
      continue;
    }
    
    /**************************************/
    /* set grid square (for beacon, wspr) */
    /**************************************/
    
    if (menu_sel == 3) {
      if (FREQFLAG == 1) {
          lcd1.home();
          lcd1.print(F("Set Grid Square "));
          FREQFLAG = 0;
      }
      if (digitalRead(vc) == 0) {
          delay(100);
          vfoChan = 3;        // tell int to ignore knob rotation
          if (digitalRead(vc) == 0) {
              for (i=0; i<4; i++)        // read current grid square from eeprom
                  gs[i] = EEPROM.read(gridAddr+i);
              gs[i]='\0';
              lcd1.home();
              lcd1.print(F("Rotate Knob    ")); 
              while (digitalRead(vc) == LOW) continue;
              delay(DEBOUNCE);
              lcd1.setCursor(0,1);        // 2nd row, char pos 0
              lcd1.print(gs);             // show current grid square from eeprom
              charPos = 0;
              lcd1.setCursor(charPos,1);
              charValue = gs[0] - 'A' + 10; // set char under cursor, offset by numbers
              lcd1.cursor();                // show cursor position
              while (true) {                // set grid, read VFO/CHAN, knob, knobsw to change values
                  if (digitalRead(vc) == 0) {    // save grid, exit routine
                      for (i=0; i<4; i++)        // save grid to eeprom
                          EEPROM.write(gridAddr+i,gs[i]);
                      lcd1.noCursor();
                      lcd1.home();
                      lcd1.print(F("Grid Saved      "));
                      lcd1.setCursor(0,1);
                      lcd1.print(F("               "));
                      lcd1.home();
                      FREQFLAG = 1;
                      break;
                  }
                  if (digitalRead(knobsw) == 0) {    // step to next char position
                      charPos++;
                      if (charPos == 4) charPos = 0;
                      if (charPos < 2) charValue = gs[charPos] - 'A' + 10; // set char under cursor, pos 0,1 alpha
                      if (charPos > 1) charValue = gs[charPos] - '0';      // set char under cursor;   // pos 2,3 numeric
                      //charValue = 0;
                      lcd1.setCursor(charPos,1);
                      lcd1.cursor();        // show current position
                      while (digitalRead(knobsw) == 0) continue;
                      delay(DEBOUNCE);
                  }
                  if ((digitalRead(knob) == LOW) && (digitalRead(knobDir) == HIGH)) {    // change char under cursor
                      charValue++;
                      if (charValue > 35) charValue = 0;
                      gs[charPos] = sChar[charValue];
                      lcd1.setCursor(charPos,1);
                      lcd1.print(gs[charPos]);
                      lcd1.setCursor(charPos,1);
                      lcd1.cursor();                // reset underline cursor
                      while (digitalRead(knob) == LOW) continue;
                      delay(DEBOUNCE);
                  }
                  if ((digitalRead(knob) == LOW) && (digitalRead(knobDir) == LOW)) {    // change char under cursor
                      charValue--;
                      if (charValue < 0) charValue = 35;
                      gs[charPos] = sChar[charValue];
                      lcd1.setCursor(charPos,1);
                      lcd1.print(gs[charPos]);
                      lcd1.setCursor(charPos,1);
                      lcd1.cursor();                // reset underline cursor
                      while (digitalRead(knob) == LOW) continue;
                      delay(DEBOUNCE);
                  }
                  
              }
              while (digitalRead(vc) == 0) continue;    // wait until vc is released
              vfoChan = 2;        // back to menu mode
              FREQFLAG = 1; // to redraw menu choice
          }
      }
                 
                      
      delay(DEBOUNCE);
      continue;
  }
 
 
     /*** Placeholder ***/
 
    if (menu_sel == 4) {
      if (FREQFLAG == 1) {
          lcd1.home();
          lcd1.print(F("menu 4           "));
          FREQFLAG = 0;
      }
      delay(DEBOUNCE);
      continue;
     }
     
   FREQFLAG = 1;
   delay(10);
   FREQFLAG = 0;
   
  }
   
}



/***************/
/**** txKey ****/
/***************/

void txKey() {  // key TX
  extern byte radioReg, rit;
  extern float freq, ritFreq;
  extern int CALOFFSET;
  float rxfreq, VALUE, DIV, VINT, VA, VB;
  VB = 1000000.0;

  // change rx osc 1 (rx freq) to 1mc during tx
  rxfreq = 1000000;    // set to 1mc, restore in main code
  VALUE = rxfreq/VB;
  DIV = XTAL * MULTI;
  VALUE = DIV/VALUE;
  VINT = (long)VALUE;
  VA = (long)((VALUE - VINT) * VB);
  clockgen.setupMultisynth(1, SI5351_PLL_A, VINT, VA, VB); // rx output on osc 1


  // set up/change the transmit frequency
  VALUE = 1;            // keep compiler happy
  if (rit == 0)
      VALUE = (freq+CALOFFSET)/VB;    // tx freq is display freq, no offsets
  if (rit == 1)
      VALUE = (ritFreq+CALOFFSET)/VB;
  DIV = XTAL * MULTI;
  VALUE = DIV/VALUE;
  VINT = (long)VALUE;
  VA = (long)((VALUE - VINT) * VB);
  clockgen.setupMultisynth(0, SI5351_PLL_A, VINT, VA, VB); // output on osc 0
 
  radioReg |= B00100000;         // set key line active high
  Wire.beginTransmission(0x20);  // set up communication with port expander
  Wire.write(0x09);              // select GPIO pins
  Wire.write(radioReg);          // update pins
  Wire.endTransmission();        // done
  
  lcd1.setCursor(15,0);
  lcd1.write("T");    // show tx mode
  
  return;
}



/*****************/
/**** txDekey ****/
/*****************/

void txDekey() {   // unkey TX, set power on for osc 0 and 2
  extern byte radioReg;
  float VALUE, DIV, VINT, VA, VB;
  VB = 1000000;
  
  radioReg &= B11011111;         // set key line active low
  Wire.beginTransmission(0x20);  // set up communication with port expander
  Wire.write(0x09);              // select GPIO pins
  Wire.write(radioReg);          // update pins
  Wire.endTransmission();        // done
  
    // set up the transmit frequency to 1 mhz (if on-freq, we hear it in rx)
    // best is to disable clock 0. Adafruit libs don't allow this. Maybe etherkit libs....
  VALUE = 1000000.0/VB;    // tx freq is 1 MHz to avoid receive interference
  DIV = XTAL * MULTI;
  VALUE = DIV/VALUE;
  VINT = (long)VALUE;
  VA = (long)((VALUE - VINT) * VB);
  clockgen.setupMultisynth(0, SI5351_PLL_A, VINT, VA, VB); // output on osc 0
  clockgen.setupMultisynth(2, SI5351_PLL_A, VINT, VA, VB); // output on osc 2
  delay(DEBOUNCE);    // sometimes the 'r' doesn't come back - unknown why
  lcd1.setCursor(15,0);
  lcd1.write("R");  // show rx mode
  return;    // note that we don't set rx freq here. That's done elsewhere.
}




/**************/
/**** SCAN ****/
/**************/

void scan() {  // in scan mode, scan 100 kc in 200hz steps. restart at end. pressing the scq 
               // switch stops and returns to the main loop. You can be in vfo or channel mode.
               // does not stop on activity, just useful to check activity.
               // Pressing and holding vc during scan pauses while held.
    extern float freq;
    float tempfreq;
    int freqMSB;
    unsigned int i;
    
    tempfreq = freq;
    while (true) {
        freq = tempfreq;   // reset to beginning
        freqMSB = (int)(freq/1000000);
        for (i=0; i<500; i++) {
            freq += 200;    //scan in increments of 200 hz
            if ((int)(freq/1000000) != freqMSB) {
                freqMSB = (int)(freq/1000000);    // get new MHz value
                updateBand();  // test for and change RF filters
            }
            updateFreq();
            updateOsc();
            if (digitalRead(vc) == LOW)        // pressing/holding vc will pause the scan
                while (digitalRead(vc) == LOW) continue;
            if (digitalRead(scq) == LOW) break;  // pressing the scq switch will stop the scan
            delay(75); // 75ms per step
        }
        if (digitalRead(scq) == LOW) break;
    }
    freq = tempfreq;
    updateFreq();
    updateBand();
    updateOsc();
    showTune();
    while (digitalRead(scq) == LOW) continue;
    delay(DEBOUNCE);
    return;
}


   
/**************************/   
/******* MAIN LOOP ********/
/**************************/

void loop() {

   /* delare vars */ 
   float tempfreq;
   extern float ritFreq;   // hold tx freq when rit is active
   extern byte rit;        // 0, no rit. 1-rit
   extern int FREQFLAG;    // 0 if no freq update, 1 if freq updated
   extern float STEP;      // tune step size in hz
   int freqMSB;            // frequency MSB (use for band register)
   extern unsigned int SIDETONE;
   int refVoltage;         // reflected voltage displayed during transmit
   int i,x;                // misc variable
   long voltLoop = 0;      // timing loop to show DC voltage updates every several seconds
   extern byte MODE;
   int modeBak = 0;
   extern int CALOFFSET;
   rit = 0;                // start with no rit
  
/* if vfo/chan button (vc) is pressed during power-up, jump
 * to MENU mode to calibrate the oscillator & change settings 
 * that don't normally get accessed.
*/
   if (digitalRead(vc) == LOW) {
     delay(DEBOUNCE);    // debounce
     if (digitalRead(vc) == LOW) {
       menu();
     }
   }
   
   // vfo/chan button - NOT pressed during startup; continue normal operation
   

   
   /*****************************/
   /* initialize radio settings */
   /*****************************/
   
   int ByHi, ByLo;
   ByHi = EEPROM.read(CalLow);    // retrieve caloffset bytes
   ByLo = EEPROM.read(CalHi);
   CALOFFSET = word(ByHi,ByLo);
   if ((CALOFFSET > 3000) || (CALOFFSET < -3000)) CALOFFSET = 0; // assume eeprom corruption or not set
   
   /* get sidetone from eeprom */
   ByLo = EEPROM.read(SidetoneLow);
   ByHi = EEPROM.read(SidetoneHi);
   SIDETONE = word(ByHi,ByLo);
   if ((SIDETONE > 1000) || (SIDETONE < 500)) SIDETONE = 700;  // assume not set or eeprom corrupted
   
   MODE = 0;         // initial define until read from the EEPROM   
   vfoChan = 0;      // start in vfo mode (vfoChan = 0)
   chan = 0;  
   Recall();         // read EEPROM from channel 0, set as vfo frequency/mode
   if ((freq < MINFREQ) || (freq > MAXFREQ)) freq = MINFREQ; // in case eeprom is corrupted or not set
   if ((MODE < 0) || (MODE > MAXMODE)) MODE = 0; // in case eeprom is corrupted or not set
   chan = 1;         // start at channel 1
   STEP = 10;        // init step size 10 hz
   updateFreq();     // initial display of frequency
   updateOsc();      // set RX oscillator frequency
   txDekey();        // set tx freq to 1MHz (start osc, but move away from rx)
   updateBand();     // set band register for low pass filters and rx filter
   updateMode();     // set MCP23008 lines and display mode
   updateDcVolt();   // show dc voltage
   freqMSB = (int)(freq/1000000);  // when this changes, update the band register
   lcd1.setCursor(15,0);
   lcd1.write("R");  // show rx mode
   showTune();       // show tuning step
   
   
   /****************************/
   /* this is the command loop */
   /****************************/
   while (1) {
     
     
     /* test rotary encoder SWITCH - change step size based on long/short push */
     if (digitalRead(knobsw) == LOW) {

        if (vfoChan == 1) {  // channel mode - increment channel number by 10. roll over at 100
          chan += 10;
          if (chan > 99) chan = 1;
          Recall();
          updateFreq();
          updateMode(); 
          updateOsc();     
          showTune();
          while (digitalRead(knobsw) == LOW) continue;
          delay(DEBOUNCE);
          continue;
        }
        
        delay(300);
        if (digitalRead(knobsw) == HIGH) {   // short press - tune 10hz, 100 hz, 1khz 
          if (STEP >= 10000) {               // at 10khz go back to 10 hz
            STEP = 10;
          } else {
             STEP *= 10;
             if (STEP > 1000)
               STEP = 10;
          }  // cycle between 10, 100, 1000 hz tune rates
          showTune();
          delay(DEBOUNCE);    // debounce after the fact 
          continue; 
        }

        if (digitalRead(knobsw) == LOW) {  // long press - tune 10K or 100K
          while (digitalRead(knobsw) == LOW){
            delay(DEBOUNCE);    // stops falsing
            continue;
          }
          if (STEP == 10000) {
            STEP = 100000;
            showTune();
            delay(DEBOUNCE);  // stops falsing
            continue;
          }
          if ((STEP < 10000) || (STEP == 100000)) {
            STEP = 10000;
            showTune();
            delay(DEBOUNCE);  // stops falsing
            continue;
          }
          delay(DEBOUNCE);    // debounce after press - stops falsing
        }
        
     }
 
   
   
     
       
     
     
     /* test for vfo/chan (short press), sto/rcl (long press) button press */
     if (digitalRead(vc) == LOW) {
         
       if (rit) {    // if rit enabled, do nothing
         while (digitalRead(vc)==0) continue;  // wait until released
         delay(DEBOUNCE);
         continue;
       }
       
       delay(300);  // check if long or short press
       
       if (digitalRead(vc) == HIGH) {    // short press
         vfoChan = abs(vfoChan - 1);
         if (vfoChan == 1) { 
           freqBak = freq;  // save vfo freq in chan mode
           modeBak = MODE;
           lcd1.noCursor();
           Recall();        // in chan mode read from EEPROM
           updateMode();
         }
         if (vfoChan == 0) { 
           freq = freqBak;  // restore vfo freq when back to vfo
           MODE = modeBak;
         }
         updateFreq();      // show vfo freq or channel #
         updateMode();
         updateOsc();
         showTune();
         continue;
       }
       
       // wait until released
       if (digitalRead(vc) == LOW) {    // long press
          
          if (vfoChan == 0) {  // in vfo mode, save to EEPROM
            Save();    // save vfo to current channel number
            lcd1.home();
            if (chan < 10) lcd1.print(F("Saved 0"));  // show channel number w/leading 0
            if (chan > 9) lcd1.print(F("Saved "));    // show 2 digit channel number
            lcd1.print(chan);
            lcd1.print(F("          "));
            while (digitalRead(vc) == LOW) continue;
            delay(700);
            updateFreq();  // show freq display
            updateMode();
            updateOsc();
            showTune();
            continue;
          }
          
          if (vfoChan == 1) {  // in channel mode recall, switch to vfo mode
            Recall();
            vfoChan = 0;
            updateFreq();
            updateMode();
            updateOsc();
            showTune();
            while (digitalRead(vc) == LOW) continue;
            delay(DEBOUNCE);
            continue;
          }
       }
       
       // something wrong
       continue;
     }
     
     
     
     /* update LCD after rotary encoder change */
     if (FREQFLAG) {    // FLAG changed due to interrupt on knob rotation
       updateFreq();    // show new freq
       updateMode();    // I2C CAN'T BE DONE IN INTERRUPT ROUTINES
       updateOsc();
       showTune();      // show tune step size
       FREQFLAG = 0;
     }
     
     
     
    /* if MHz digit changes, test and update the band register */ 
    if ((int)(freq/1000000) != freqMSB) {
      freqMSB = (int)(freq/1000000);    // get new MHz value
      updateBand();  // test for and change RF filters
    }
     
     
     
    /* test keyIn line (manual key for cw, ptt for ssb) */
    
    if (digitalRead(keyIn1) == 0) {    // TX key pressed 
      if (MODE > 3) continue;          // only keyup for cw/ssb modes
      delay(10);                       // debounce the key
      if ((MODE == 2) || (MODE == 3))    
          tone(toneOut, SIDETONE);     // turn on sidetone for cw modes
      txKey();                         // key the transmitter
      while (digitalRead(keyIn1)==0) { 
        refVoltage = analogRead(refIn);  // read reflected power
        refVoltage /= 128;               // bring in range 0-7
        lcd1.setCursor(0,1);             // print a bar graph of ref power
        for (i=0; i<refVoltage; i++) lcd1.write('|');
        for (x=i; x<8; x++) lcd1.write(' ');  // blank rest of display
        continue;                     // wait 'till released
      }
      // cw key/ptt released
      if ((MODE == 2) || (MODE == 3))
          noTone(toneOut);           // turn off sidetone
      txDekey();                     // dekey the transmitter
      updateOsc();                   // restore rx freq (set to 1mc during tx)
      updateFreq();                  // restore the display from swr reading
      updateMode();                  // display update
      updateDcVolt();                // display update
      if (rit) {
         lcd1.setCursor(5,1);
         lcd1.print(F("RIT"));       // display update
      }
      showTune();                    // display update
      continue;
    }
     
     
    /* test mode/rit button */
    if (digitalRead(mr) == 0) {
     delay(300);                    // test for long/short press
     if (digitalRead(mr) == 1) {    // short press - MODE function
        if (rit) continue;          // don't change modes in rit mode  
        MODE += 1;                  // change to next mode 
        if (MODE > MAXMODE-1) MODE = 0;      // cycle thru
        updateMode();               // update LCD/radio registers 
        updateOsc();                // switch rx offset based on mode
        showTune();
        continue;
     }
     // Long press - enable split (RIT) (still pressed)
     if (vfoChan == 1) {            // in chan mode do nothing
         while (digitalRead(mr) == 0) continue;
         delay(DEBOUNCE);
         continue;
     }
     rit = abs(rit-1);
     if (rit) {
         ritFreq = freq;           // turn ON rit
         lcd1.setCursor(5,1);
         lcd1.print(F("RIT"));
         STEP = 10;                // in rit, init step size as smallest step
         showTune();
     } else {
         freq = ritFreq;          // turn OFF rit
         updateFreq();            // restore rx freq
         updateBand();
         updateOsc();
         lcd1.setCursor(5,1);
         lcd1.print(F("   "));
         showTune();
     }
     while (digitalRead(mr) == 0) continue;
     delay(DEBOUNCE);
    }  // done
    
    
    
    /* test scan/send cq button (scan - long press, send cq/wspr/beacon - short press)  */
    if (digitalRead(scq) == LOW) {
        delay(300);                // test for short/long press
        if (digitalRead(scq) == HIGH) {                  // short press - beacon modes/cq mode
            if ((MODE == 2) || (MODE == 3)) sendCQ();    // CW mode, send CQ
            if (MODE == 4) {       // WSPR mode, transmit from the displayed freq
                tempfreq = freq;
                wspr(freq);  
                freq = tempfreq;
            }
            if (MODE == 5) beacon();  // Beacon mode, transmit from the displayed freq
            // ignore SSB modes
            updateFreq();
            updateBand();
            updateOsc();
            showTune();
            continue;  
        }
        if (digitalRead(scq) == LOW) {    // long press - scan
            while (digitalRead(scq) == LOW) continue;
            delay(DEBOUNCE);
            scan();
            delay(DEBOUNCE);
            showTune();
            freqMSB = (int)(freq/1000000);    // get new MHz value (may have changed during scan)
            continue;
        }
    }
   
            
    

    /* update the DC voltage reading once every 5 seconds or so */
     voltLoop++;
     if (voltLoop == 90000) {  // 90,000 is roughly 5 seconds at 16 MHz clock
         updateDcVolt();
         showTune();
         voltLoop = 0;
     }

   
     
    continue;  // end of main loop
   }
}

/***********************************************/
/************* End of Main Code ****************/
/***********************************************/


/***********************************************************/
/*** Set Default values, frequencies and modes in EEPROM ***/
/***********************************************************/

void setDefault() {  /* initialize the EEPROM with default frequencies */

  // NOTE: this is ONLY called to initialize the EEPROM
  // (and may not be called at all - user's choice)
  // Also Note: these memory slots are overwritten when you store channels. 
  
  extern float freq;
  extern int chan;
  extern byte MODE;
  //int i;
  
  // EEPROM storage: frequency (4 bytes), mode (1 byte)
  
  // I'd use PROGMEM here but it doesn't work inside functions
  // and I don't want all this cluttering up the beginning
  
  /* NOTE: Channels can contain frequencies above/below MINFREQ/MAXFREQ. */
  /* (You just won't be able to tune them if recalled to VFO) (see operation.txt) */
  
  const float defaultFreq[100] = {
     7030000,  // ch 00, start freq/mode when turned on
     3525000,  // 80M cw ch 1
     7035000,  // 40M cw ch 2
    10110000,  // 30M cw ch 3
    14035000,  // 20M cw ch 4
    
     3825000,  // 80M ssb ch 5
     7240000,  // 40M ssb ch 6
    14275000,  // 20M ssb ch 7
   
     3570100,  // WSPR 80M  ch 8
     7040100,  // WSPR 40M  ch 9
    10140200,  // WSPR 30M  ch 10
    14097100,  // WSPR 20M  ch 11
   
     5000000,  // WWV 5 mc  ch 12
    10000000,  // WWV 10 mc ch 13
    15000000,  // WWV 15 mc ch 14
     3330000,  // CHU 3 mc  ch 15
     7850000,  // CHU 7 mc  ch 16
    14670000,  // CHU 14 mc ch 17
    
     3485000,  // volmet US, Can ch 18
     6604000,  // volmet ""  ""  ch 19
    10051000,  // volmet "" ""   ch 20
    13270000,  // volmet "" ""   ch 21
     6754000,  // volmet Trenton ch 22
    15034000,  // volmet   ""    ch 23
     3413000,  // volmet Europe  ch 24
     5505000,  // volmet   ""    ch 25
     8957000,  // volmet   ""    ch 26
    13270000,  // volmet   ""    ch 27
     5450000,  // volmet Europe  ch 28
    11253000,  // volmet   ""    ch 29
     4742000,  // volmet Europe  ch 30
    11247000,  // volmet   ""    ch 31
     6679000,  // volmet Oceania ch 32
     8828000,  // volmet   ""    ch 33
    13282000,  // volmet   ""    ch 34
     6676000,  // volmet SE Asia ch 35
    11387000,  // volmet   ""    ch 36
     3458000,  // volmet   ""    ch 37
     5673000,  // volmet   ""    ch 38
     8849000,  // volmet   ""    ch 39
    13285000,  // volmet   ""    ch 40
 
     4426000,  // USCG Weather   ch 41
     6501000,  //       ""       ch 42
     8764000,  //       ""       ch 43
    13089000,  //       ""       ch 44
     4316000,  //       ""       ch 45
     8502000,  //       ""       ch 46
    12788000,  //       ""       ch 47
     4125000,  // USCG Distress  ch 48
     6215000,  //       ""       ch 49
     8291000,  //       ""       ch 50
    12290000,  //       ""       ch 51
 
     8992000,  // USAF HFGCS     ch 52
    11175000,  // USAF   ""      ch 53
     6739000,  // USAF   ""      ch 54
      
     8152000,  //  Marine chat   ch 55
    10000000,  //  Placeholder   ch 56
    10000000,  //      ""        ch 57
    10000000,  //      ""        ch 58
    10000000,  //      ""        ch 59
    50081000,  //  50 Mhz Beacon ch 60
    
     5330500,  // US Ham 60M ch1 ch 61
     5346500,  // US Ham 60M ch2 ch 62
     5357500,  // US Ham 60M ch3 ch 63
     5371500,  // US Ham 60M ch4 ch 64
     5403500,  // US Ham 60M ch5 ch 65
  

    10000000,  // Placeholder    ch 66
    11000000,  //      ""        ch 67
    12000000,  //      ""        ch 68
    13000000,  //      ""        ch 69
    14000000,  //      ""        ch 70
    15000000,  //      ""        ch 71
    10000000,  //      ""        ch 72
    10000000,  //      ""        ch 73
    10000000,  //      ""        ch 74
    10000000,  //      ""        ch 75
    10000000,  //      ""        ch 76
    10000000,  //      ""        ch 77
    10000000,  //      ""        ch 78
    10000000,  //      ""        ch 79
    10000000,  //      ""        ch 80
    10000000,  //      ""        ch 81
    10000000,  //      ""        ch 82
    10000000,  //      ""        ch 83
    10000000,  //      ""        ch 84
    10000000,  //      ""        ch 85
    10000000,  //      ""        ch 86
    10000000,  //      ""        ch 87
    10000000,  //      ""        ch 88
    10000000,  //      ""        ch 89
    10000000,  //      ""        ch 90
    10000000,  //      ""        ch 91
    10000000,  //      ""        ch 92
    10000000,  //      ""        ch 93
    10000000,  //      ""        ch 94
    10000000,  //      ""        ch 95
    10000000,  //      ""        ch 96
    10000000,  //      ""        ch 97
    10000000,  //      ""        ch 98
    10000000   // (place holder) ch 99    
  };
  
  /* set the mode for the memory channels */
  // 0=lsb, 1=usb, 2=cw-upper sideband, 3=cw-lower sideband, 4=WSPR, 5=Beacon
  
    byte defaultMode[100] = {
    3,3,3,3,3,          // ch 00 - 04 
    0,0,1,4,4,4,4,      // ch 05 - 11
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,  // 12 - 25
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  // 26 - 50
    1,1,1,1,1,1,1,1,1,5,  // 51 - 60
    1,1,1,1,1,         // 61 - 65 (usb)
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  // 66 - 80
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1   // 81 - 99
  };
    
  
  // now load these channels in EEPROM
  MODE = 1;  // defaults to USB
  for (chan=0; chan<100; chan++) { // includes ch 00
     freq = defaultFreq[chan];
     MODE = defaultMode[chan];
     Save();
     lcd1.setCursor(0,0);    // watch as the channels fill
     lcd1.print(F("CHAN "));
     lcd1.print(chan);
     delay(10);
  }
  

  // set CALOFFSET to 0  (you need to set correct value from menu)
  EEPROM.write(CalLow, lowByte((int)0));
  EEPROM.write(CalHi, highByte((int)0));
  
  // save init sidetone value
  EEPROM.write(SidetoneHi, highByte((int)700));
  EEPROM.write(SidetoneLow, lowByte((int)700));
  
  // save default grid square
  EEPROM.write(gridAddr+0,'A');    // save grid to eeprom
  EEPROM.write(gridAddr+1,'B');    // yeah, it's ugly. Running out of ram
  EEPROM.write(gridAddr+2,'0');    // and this gets the job done.
  EEPROM.write(gridAddr+3,'1');
  
  // done
  return;
}





/**********************************/
/********* WSPR Transmitter *******/
/**********************************/

void wspr(float wfreq) { // (frequency sent from calling routine)

  /* For wspr to work correctly transmission must start at 1 second
     into an even UTC minute. This must be done manually in this software.

     Much of the symbol generation was lifted from the excellant work 
     of Mark VandeWetterin at https://github.com/brainwagon/genwspr
     The c code as written does not work on the arduino (it wasn't meant to). 
     Some changes needed to be made (change ints to long, unsigned char 
     to byte, etc) and I added the bits needed to make it transmit. - kurt
  */

/********* START of encode routine *********/



byte rdx[]  = {   // note byte in place of char
    0, 128, 64, 32, 160, 96, 16, 144, 80, 48, 112, 8, 136, 72, 40, 104, 24,
    152, 88, 56, 120, 4, 132, 68, 36, 100, 20, 148, 84, 52, 116, 12, 140,
    76, 44, 108, 28, 156, 92, 60, 124, 2, 130, 66, 34, 98, 18, 146, 82, 50,
    114, 10, 138, 74, 42, 106, 26, 154, 90, 58, 122, 6, 134, 70, 38, 102,
    22, 150, 86, 54, 118, 14, 142, 78, 46, 110, 30, 158, 94, 62, 126, 1,
    129, 65, 33, 161, 97, 17, 145, 81, 49, 113, 9, 137, 73, 41, 105, 25,
    153, 89, 57, 121, 5, 133, 69, 37, 101, 21, 149, 85, 53, 117, 13, 141,
    77, 45, 109, 29, 157, 93, 61, 125, 3, 131, 67, 35, 99, 19, 147, 83, 51,
    115, 11, 139, 75, 43, 107, 27, 155, 91, 59, 123, 7, 135, 71, 39, 103,
    23, 151, 87, 55, 119, 15, 143, 79, 47, 111, 31, 159, 95, 63, 127 
};

  byte msg[162];

  int n,i;
  // int xx;     // debug code
  unsigned long time;
  
  //float shift = 12000.0/8192.0;  // this is the ideal way to do it
  float shift = 1.65;    // account for freq variation in si5351 library
  
  //More accurate: int txtime = (int)((float)((8192/12000)*1000.0));
  int txtime = 683;  // time in msec (actully 1/shift * 1000 (for msec))
  
  extern float freq;
  
  // encode the call sign
 long c = encodecallsign(CALLSIGN);
 
  // encode the grid square from eeprom
  char gsq[5];
  for (i=0; i<4; i++)        // read grid from eeprom
      gsq[i] = EEPROM.read(gridAddr+i);
      gsq[i]='\0';
  long g = encodegrid(gsq);
  
  // encode the power (dBm)
  long p = encodepower(RFPOWER);  // was 10
 
  int mp = 0 ;
  unsigned long acc = 0;

  // initialize message
  for (i=0; i<162; i++) msg[i] = pgm_read_byte_near(sync + i);    // was sync[i]
	
 
    /* by default, arduino does not do math well enough (think k&r c from '68) */
  for (i=27; i>=0; i--) {		/* encode the callsign, 28 bits */
    acc <<= 1;
    if (c & 1L<<i) acc |= 1;          // NOTE the 1L (damn 8 bit thinking...)
    msg[rdx[mp++]] += 2*parity(acc & 0xf2d05351L);
    msg[rdx[mp++]] += 2*parity(acc & 0xe4613c47L);
  }

  for (i=14; i>=0; i--) {		/* encode the grid, 15 bits */
    acc <<= 1;
    if (g & (1<<i)) acc |= 1;
    msg[rdx[mp++]] += 2*parity(acc & 0xf2d05351L);
    msg[rdx[mp++]] += 2*parity(acc & 0xe4613c47L);
  }

  for (i=6; i>=0; i--) {		/* encode the power, 7 bits */
    acc <<= 1;
    if (p & (1<<i)) acc |= 1;
    msg[rdx[mp++]] += 2*parity(acc & 0xf2d05351L);
    msg[rdx[mp++]] += 2*parity(acc & 0xe4613c47L);
  }

  for (i=30; i>=0; i--) {		/* pad with 31 zero bits */
    acc <<= 1L;
    msg[rdx[mp++]] += 2*parity(acc & 0xf2d05351L);
    msg[rdx[mp++]] += 2*parity(acc & 0xe4613c47L);
  }
    
    /* we have the 162 byte code in msg[], send it */
    
  lcd1.home();
  lcd1.print(F("Sending          "));
  // xx=0;        // debug code
  for (i=0; i<162; i++) {
    n = msg[i]; 
    /*
    Serial.print(n);   // debug code - remove from production
    Serial.print(" ");
    xx++; 
    if (xx>17) {
        xx=0;
        //Serial.print("\n");
    }
    */
    freq = wfreq + ((float)n*shift);
    txKey();  // freq got updated, call key routine to change on the fly
    time = millis() + txtime;
    while (millis() < time) continue; // wait until 0.683 seconds pass
    if (digitalRead(scq) == LOW) {    // scq switch pressed - exit wspr routine (abort)
        txDekey();
        while (digitalRead(scq) == LOW) continue;
        delay(DEBOUNCE);
        return;
    }
  }
  txDekey();
  return;
}


/********** WSPR Calculations Below ***********/

// lots of longs where int was specified to allow for arduino 8 bit math

int chval1(int ch) {
    if (isdigit(ch)) return ch - '0';
    if (isalpha(ch)) return 10 + toupper(ch) - 'A';
    if (ch == ' ') return 36;
    return 0;    // keep compiler happy - will never reach here
}

int chval2(int ch) {
    if (isalpha(ch)) return toupper(ch) - 'A';
    if (ch == ' ') return 26;
    return 0;    // keep compiler happy - will never reach here
}


long encodecallsign(const char *callsign) { // const
    /* find the first digit... */
    unsigned int i;
    long rc;
    char call[6];

    for (i=0; i<6; i++) call[i] = ' ';

    if (isdigit(callsign[1])) {
	/* 1x callsigns... */
	for (i=0; i<strlen(callsign); i++)
	   call[1+i] = callsign[i];
    } else if (isdigit(callsign[2])) {
	/* 2x callsigns... */
	for (i=0; i<strlen(callsign); i++)
	   call[i] = callsign[i];
    } else {
	return 0;
    }

    rc  = chval1(call[0]); rc *= 36; 
    rc += chval1(call[1]); rc *= 10;
    rc += chval1(call[2]); rc *= 27;
    rc += chval2(call[3]); rc *= 27;
    rc += chval2(call[4]); rc *= 27;
    rc += chval2(call[5]);

    return rc;
}

long encodegrid(const char *grid) {
    long rc;

    rc = (179 - 10 * (grid[0]-'A') - (grid[2] - '0')) * 180
	 + (10 * (grid[1]-'A')) + (grid[3] - '0');

    return rc;
}

int encodepower(const int p) {
    return p + 64;
}

int parity(unsigned long x) {  // returns 1 or 0, x is a BIG #
    int even = 0;
    while (x) {
	even = 1-even; 
	x = x & (x - 1);
    }
    return even;
}


/*****************/
/**** Send CQ ****/
/*****************/

void sendCQ() { // send a cq message 
    unsigned int i;
    char cwstg[] = "CQ CQ CQ DE ";    // If changed, use UPPER CASE
    for (i=0; i<strlen(cwstg); i++) sendCw(cwstg[i]); // strlen
    for (i=0; i<strlen(call); i++) sendCw(call[i]);      // send call
    sendCw(' ');
    for (i=0; i<strlen(call); i++) sendCw(call[i]);      // send call
    sendCw(' ');
    sendCw('K');    // Use UPPER CASE
    return;
}



/******************/
/****  BEACON  ****/
/******************/

void beacon() {    // send 8 seconds of carrier followed by CW ID. Runs until scq switch pressed at most 1/2 second
    
    unsigned int i; // strlen returns unsigned int
    int oldVfoChan;    
    char cwstg1[] = "DE ";    // UPPER CASE for both
    char cwstg2[] = " BEACON";
    oldVfoChan = vfoChan;    // save existing
    vfoChan = 3;             // ignore knob rotation 
    while (true) {
        txKey();
        for (i=0; i<16; i++) {    // send 16 1/2 second sequential transmissions
            txKey();              // (if we used 1 16 second delay, we can't abort)
            delay(500);
            if (digitalRead(scq) == LOW) {    // switch press ends beacon, resumes normal op
                txDekey();
                while (digitalRead(scq) == LOW) continue;
                vfoChan = oldVfoChan;        // restore settings
                return;
            }
        }
        txDekey();
        delay(1000);          // 1 second between carrier and ID
        for (i=0; i<strlen(cwstg1); i++) sendCw(cwstg1[i]);      // send DE
        for (i=0; i<strlen(call); i++) sendCw(call[i]);          // send call
        sendCw(' ');                                             // space between call/grid
        for (i=0; i<4; i++) sendCw(EEPROM.read(gridAddr+i));     // send grid
        for (i=0; i<strlen(cwstg2); i++) sendCw(cwstg2[i]);      // send BEACON
        delay(1000);         // 1 second between ID and carrier
        continue;            // continue with 8 seconds carrier followed by ID
    }
} 



/*************************************/
/* Transmit a single character as CW */
/*************************************/

void sendCw(char cwch) { 

// initialisation at start of code for this routine (global vars in flash mem)

    // CW Speed is (1.2/WPM) * 1000 
    int SPEED = 15;    // speed is WPM
    int DLY = (1.2/SPEED)*1000;    
    unsigned int x, charnum;
    char dly;
    while (digitalRead(knobsw) == LOW) continue;
    delay(DEBOUNCE);
    
 
    if (cwch == '\0') return;
    
    if (cwch == ' ') {    
        // is a space
        delay(DLY*3);
        return;
    }
    
    // is a number
    if (isDigit(cwch)) {
        charnum = cwch - '0';
        for (x=0; x<6; x++) {
            dly = pgm_read_byte_near(numCw[charnum]+x);    // use flash to store values
            if (dly == '0') {    // terminating char is 0 - delay only
                delay(DLY*3);    // delay 3 bit lengths between chars
                continue;
            }
            txKey();
            if (dly == '1') delay(DLY*1);    // delay 1 bit length
            if (dly == '3') delay(DLY*3);    // delay 3 bit lengths
            txDekey();
            delay(DLY*1);    // delay 1 bit length 
            continue;
        }
    }
    
    // is a letter
    if (isAlpha(cwch)) {
        charnum = cwch - 'A';
        for (x=0; x < strlen_P(chrCw[charnum]); x++) {
            dly = pgm_read_byte_near(chrCw[charnum]+x);    // read data from flash
            if (dly == '0') {    // terminating char is 0 - delay only
                delay(DLY*3);    // delay 3 bit lengths between chars
                continue;
            }
            txKey();
            if (dly == '1') delay(DLY*1);    // delay 1 bit length
            if (dly == '3') delay(DLY*3);    // delay 3 bit lengths
            txDekey();
            delay(DLY*1);    // delay 1 bit length
            continue;
        }
    }
    // char is something unknown or routine is finished

    return;
}
       


