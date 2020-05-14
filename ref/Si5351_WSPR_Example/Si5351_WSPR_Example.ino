// Si5351_WSPR
//
// Simple WSPR beacon for Arduino Uno, with the Etherkit Si5351A Breakout
// Board, by Jason Milldrum NT7S.
// 
// Original code based on Feld Hell beacon for Arduino by Mark 
// Vandewettering K6HX, adapted for the Si5351A by Robert 
// Liesenfeld AK6L <ak6l@ak6l.org>.  Timer setup
// code by Thomas Knutsen LA3PNA.
//
// Time code adapted from the TimeSerial.ino example from the Time library.

// Hardware Requirements
// ---------------------
// This firmware must be run on an Arduino AVR microcontroller
//
// Required Libraries
// ------------------
// Etherkit Si5351 (Library Manager)
// Etherkit JTEncode (Library Manager)
// Time (Library Manager)
// Wire (Arduino Standard Library)
//
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#include <si5351.h>
#include <JTEncode.h>
#include <int.h>
#include <TimeLib.h>

#include "Wire.h"

#define TONE_SPACING            146           // ~1.46 Hz
#define WSPR_CTC                10672         // CTC value for WSPR
#define SYMBOL_COUNT            WSPR_SYMBOL_COUNT
#define CORRECTION              0             // Change this for your ref osc

#define TIME_HEADER             "T"           // Header tag for serial time sync message
#define TIME_REQUEST            7             // ASCII bell character requests a time sync message 

#define TX_LED_PIN              12
#define SYNC_LED_PIN            13


// Global variables
Si5351 si5351;
JTEncode jtencode;
unsigned long freq = 10140200UL;                // Change this
char call[7] = "N0CALL";                        // Change this
char loc[5] = "AB12";                           // Change this
uint8_t dbm = 10;
uint8_t tx_buffer[SYMBOL_COUNT];

// Global variables used in ISRs
volatile bool proceed = false;

// Timer interrupt vector.  This toggles the variable we use to gate
// each column of output to ensure accurate timing.  Called whenever
// Timer1 hits the count set below in setup().
ISR(TIMER1_COMPA_vect)
{
    proceed = true;
}
 
// Loop through the string, transmitting one character at a time.
void encode()
{
    uint8_t i;

    jtencode.wspr_encode(call, loc, dbm, tx_buffer);
    
    // Reset the tone to 0 and turn on the output
    si5351.set_clock_pwr(SI5351_CLK0, 1);
    digitalWrite(TX_LED_PIN, HIGH);
    
    // Now do the rest of the message
    for(i = 0; i < SYMBOL_COUNT; i++)
    {
        si5351.set_freq((freq * 100) + (tx_buffer[i] * TONE_SPACING), SI5351_CLK0);
        proceed = false;
        while(!proceed);
    }
        
    // Turn off the output
    si5351.set_clock_pwr(SI5351_CLK0, 0);
    digitalWrite(TX_LED_PIN, LOW);
}

void processSyncMessage()
{
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER))
  {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME)
     { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}
 
void setup()
{
  // Use the Arduino's on-board LED as a keying indicator.
  pinMode(TX_LED_PIN, OUTPUT);
  pinMode(SYNC_LED_PIN, OUTPUT);
  
  digitalWrite(TX_LED_PIN, LOW);
  digitalWrite(SYNC_LED_PIN, LOW);
  Serial.begin(9600);

  // Set time sync provider
  setSyncProvider(requestSync);  //set function to call when sync required
      
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, CORRECTION);

  // Set CLK0 output
  si5351.set_freq(freq * 100, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock initially
  
  // Set up Timer1 for interrupts every symbol period.
  noInterrupts();          // Turn off interrupts.
  TCCR1A = 0;              // Set entire TCCR1A register to 0; disconnects
                           //   interrupt output pins, sets normal waveform
                           //   mode.  We're just using Timer1 as a counter.
  TCNT1  = 0;              // Initialize counter value to 0.
  TCCR1B = (1 << CS12) |   // Set CS12 and CS10 bit to set prescale
    (1 << CS10) |          //   to /1024
    (1 << WGM12);          //   turn on CTC
                           //   which gives, 64 us ticks
  TIMSK1 = (1 << OCIE1A);  // Enable timer compare interrupt.
  OCR1A = WSPR_CTC;       // Set up interrupt trigger count;
  interrupts();            // Re-enable interrupts.
}
 
void loop()
{
  if(Serial.available())
  {
    processSyncMessage();
  }
  
  if(timeStatus() == timeSet)
  {
    digitalWrite(SYNC_LED_PIN, HIGH); // LED on if synced
  }
  else
  {
    digitalWrite(SYNC_LED_PIN, LOW);  // LED off if needs refresh
  }

  // Trigger every 10th minute
  // WSPR should start on the 1st second of the minute, but there's a slight delay
  // in this code because it is limited to 1 second resolution.
  if(timeStatus() == timeSet && minute() % 10 == 0 && second() == 0)
  {
    encode();
    delay(1000);
  }
  
  //delay(100);
}
