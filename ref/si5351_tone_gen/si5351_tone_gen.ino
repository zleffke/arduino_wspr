//
// Simple JT65/JT9/JT4/FT8/WSPR/FSQ beacon for Arduino, with the Etherkit
// Si5351A Breakout Board, by Jason Milldrum NT7S.
//
// Transmit an abritrary message of up to 13 valid characters
// (a Type 6 message) in JT65, JT9, JT4, a type 0.0 or type 0.5 FT8 message,
// a FSQ message, or a standard Type 1 message in WSPR.
//
// Connect a momentary push button to pin 12 to use as the
// transmit trigger. Get fancy by adding your own code to trigger
// off of the time from a GPS or your PC via virtual serial.
//
// Original code based on Feld Hell beacon for Arduino by Mark
// Vandewettering K6HX, adapted for the Si5351A by Robert
// Liesenfeld AK6L <ak6l@ak6l.org>.
//
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
#include <int.h>
#include <string.h>

#include "Wire.h"


// Hardware defines
#define BUTTON        12
#define RED_LED       13
#define GREEN_LED     8

// Class instantiation
Si5351 si5351;

// Global variables
unsigned long       freq = 7040100UL;
  signed long correction = 15300UL;

uint8_t led_state = 0;
uint8_t tx_state = false;

volatile bool btn_press  = false; //set in button ISR

void setup()
{
  // initialize digital pin outputs.
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  pinMode(BUTTON, INPUT_PULLUP); // Use a button connected to pin 12 as a transmit trigger

  //Setup interrupts
  int irq_btn = digitalPinToInterrupt(BUTTON);
  attachInterrupt(irq_btn, btn_isr, FALLING);
  
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, correction);
  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially

}

void loop()
{
  if (btn_press){ //ARM For TX on next time interval.  Every two minutes.
    btn_press = false;
    tx_state = ~tx_state;
    Serial.println(tx_state);
    set_state_tx(tx_state);
    //Serial.println(tx_arm);
    //digitalWrite(GREEN_LED, tx_arm);
    toggle_led();
  }
  
}

void set_state_tx(bool state){
  if (state) {
    digitalWrite(RED_LED, HIGH);
    si5351.output_enable(SI5351_CLK0, 1);
    si5351.set_freq((freq * 100), SI5351_CLK0);
  }
  else{
    digitalWrite(RED_LED, LOW);
    si5351.output_enable(SI5351_CLK0, 0);
  }
}

void btn_isr(){
  //SerialUSB.println("THERMO ISR Fired!!!");
  delay(50);
  if(digitalRead(BUTTON) == LOW){
    delay(100);   // delay to debounce
    if (digitalRead(BUTTON) == LOW){
      btn_press = true;
      delay(100); //delay to avoid extra triggers
    }
  }
}

void toggle_led(){
  led_state = ~led_state;
  digitalWrite(RED_LED, led_state);
  digitalWrite(GREEN_LED, led_state);
}
