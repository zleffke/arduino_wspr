#include "Arduino.h"
#include <si5351.h>
#include <JTEncode.h>
#include <rs_common.h>
//#include <int.h>
#include <string.h>
#include <Adafruit_GPS.h> // Adafruit Ultimate GPS Library; https://github.com/adafruit/Adafruit_GPS
#include <TimeLib.h>      // Time functions  https://github.com/PaulStoffregen/Time
#include "Wire.h"

//****Timer Defines*********************************
//#define CPU_HZ 48000000
//#define TIMER_PRESCALER_DIV 1024
//uint32_t sampleRate = 682; //sample rate in milliseconds, determines how often TC5_Handler is called
uint32_t sampleRate = 682; //sample rate in milliseconds, determines how often TC5_Handler is called

//****Board Pinouts*********************************
//OUTPUT
#define TX_LED1       5
#define TX_LED2       A0
#define RED_LED       13
#define GREEN_LED     8
#define GPS_RESET     A3 //RESET by pulling low
#define GPS_FIX       A4 //Same as GPS LED Drive, when fix pulses high every 15sec, when no fix pulses high every 1 sec
#define GPS_PPS       11 //pulses high, usually low
#define GPS_ENABLE    A5 //Active LOW, pull high to DISABLE GPS
#define BUTTON        12
#define XTAL_TEMP     A1

// ****WSPR Details********************
#define WSPR_TONE_SPACING       146          // ~1.46 Hz
//#define WSPR_DELAY              683          // Delay value for WSPR
#define WSPR_DELAY              681          // slightly less to account for if statement in TX state
#define WSPR_BAUD               1500        //baud * 1000 corrected for delays in interrupt routines.
#define WSPR_DEFAULT_FREQ       14097200UL
#define FREQ_CORRECTION         15300UL
#define DEFAULT_MODE            MODE_WSPR

typedef struct band{
  uint32_t b160=  1838100UL;
  uint32_t b80 =  3570100UL;
  uint32_t b40 =  7040100UL;
  uint32_t b30 = 10140200UL;
  uint32_t b20 = 14097100UL;
  uint32_t b17 = 18106100UL;
  uint32_t b15 = 21096100UL;
  uint32_t b12 = 24926100UL;
  uint32_t b10 = 28126100UL;
}band;

band wspr_band;

uint32_t freq = wspr_band.b30; //default to 30m band
uint8_t band_idx = 0;


// Class instantiation
Si5351 si5351;
JTEncode jtencode;

//--------GPS Details--------------
// what's the name of the hardware serial port?
#define GPSSerial Serial1
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Global variables
//unsigned long freq;
char message[] = "KJ4QLP EM97";
char call[] = "KJ4QLP";
char loc[] = "EM97";
uint8_t dbm = 10;
uint8_t tx_buffer[255];
uint8_t tx_idx = 0; //trasnmit buffer index
bool symbol_step = false; //if true, increment symbol index

uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;
//*****Time Variables********
#define SYNC_INTERVAL          10 // time, in seconds, between GPS sync attempts
#define SYNC_TIMEOUT           60 // time(sec) without GPS input before error
//time_t syncTime = 0; // time of last GPS or RTC synchronization
time_t  pps_now = 0; //'now' time when PPS fires
//time_t gps_time = 0; //Set GPS Time
//unsigned long syncMillis = 0; // millis() at sync
//unsigned long timeSinceSync = 0;
bool synced     = false; //Time synchronized yet?
uint32_t tx_start = 0;
uint32_t tx_stop  = 0;
uint32_t tx_duration = 0;
float xtal_temp = 0;


//uint32_t      timer      = millis();
uint8_t       led_state  = 0;
//bool resync = false;

uint8_t tx_arm = 0;
bool tx_state = false;
bool next_pps_tx = false;

volatile bool btn_press  = false; //set in button ISR
volatile bool pps_fired  = false; //set in PPS ISR


void setup()
{
  Serial.begin(115200);
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz

  // initialize digital pin outputs.
  pinMode(TX_LED1, OUTPUT);
  digitalWrite(TX_LED1, LOW);
  pinMode(TX_LED2, OUTPUT);
  digitalWrite(TX_LED2, LOW);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH);
  pinMode(GPS_ENABLE, OUTPUT);
  digitalWrite(GPS_ENABLE, LOW); //ENABLE GPS
  //initialize digital pin inputs
  pinMode(GPS_FIX, INPUT);
  pinMode(GPS_PPS, INPUT);
  pinMode(BUTTON, INPUT_PULLUP); // Use a button connected to pin 12 as a transmit trigger
  analogReadResolution(12);

  //Setup interrupts
  int irq_pps = digitalPinToInterrupt(GPS_PPS);
  attachInterrupt(irq_pps, pps_isr, RISING);
  int irq_btn = digitalPinToInterrupt(BUTTON);
  attachInterrupt(irq_btn, btn_isr, FALLING);

  
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, FREQ_CORRECTION);
  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially


  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  //freq = WSPR_DEFAULT_FREQ;
  //freq = wspr_band.b40;
  symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
  tone_spacing = WSPR_TONE_SPACING;
  tone_delay = WSPR_DELAY;

  // Encode the message in the transmit buffer
  // This is RAM intensive and should be done separately from other subroutines
  set_tx_buffer();
  //startTimer(1.4684);

  //Setup Timer and start
  tcConfigure(WSPR_BAUD); //configure the timer to run at <sampleRate>Hertz
  //tcStartCounter(); //starts the timer
  tcDisable();
}

void loop(){
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another      
    set_gps_time(GPS);
  }
  if (btn_press){ //ARM For TX on next time interval.  Every two minutes.
    btn_press = false;
    tx_arm = ~tx_arm;
    //Serial.println(tx_arm);
    if (tx_arm){
      digitalWrite(GREEN_LED, tx_arm);
      delay(100); //extra padding
      Serial.println("Armed For Transmit");
    }
    else{
      digitalWrite(GREEN_LED, tx_arm);
      Serial.println("DISARMED");
    }
    if (tx_state){
      Serial.println("Terminating Transmission, Button");
      SetStateStandby();
    }
    //toggle_led();
    //encode();
  }
    
  if (pps_fired){
    pps_now = now();
    pps_fired = false; //reset PPS flag
    if (next_pps_tx){
      next_pps_tx = false;
      SetStateTX();
    }
    
    if (!tx_state){
      get_xtal_temp();
      //SyncCheck(pps_now);
      print_sys_time(pps_now);
      if (tx_arm){ //not transmitting and armed
        uint8_t s = second(pps_now);
        uint8_t m = minute(pps_now);      
        if (synced && s==0 && m%2==0){
          Serial.println("Transmitting on Next PPS");
          if      (m%10==0){freq=wspr_band.b80;}
          else if (m%10==2){freq=wspr_band.b40;}
          else if (m%10==4){freq=wspr_band.b30;}
          else if (m%10==6){freq=wspr_band.b20;}
          else if (m%10==8){freq=wspr_band.b17;}
          Serial.print("Set TX Freq: "); Serial.println(freq);
          //print_gps_time(GPS);
          //SetStateTX();
          next_pps_tx=true;
        }
      }    
    }
    else{
      uint8_t s = second(pps_now);
      if (s%10 == 0){
        print_sys_time(pps_now);
        get_xtal_temp();
      }
    }
  }
  
  if (tx_state){
    if (symbol_step){
      toggle_led();
      symbol_step = false;
      if (tx_idx<symbol_count){
        //Serial.println(tx_idx);
        si5351.set_freq((freq * 100) + (tx_buffer[tx_idx] * tone_spacing), SI5351_CLK0);
      }
      else{
        Serial.println("Transmission Completed Successfully!");
        SetStateStandby();
      }
    }
  }
}

float get_xtal_temp()
{
  float adc_cal_val = -0.035;
  float adc_val = analogRead(XTAL_TEMP);
  float adc_volts = adc_val / 4096 * 3.3 + adc_cal_val;
  xtal_temp = 100.0 * adc_volts - 50.0; //C
  //xtal_temp = (adc_volts - V0)/Tc;
  
  
  //val * 0.0625;
  Serial.print(adc_val); Serial.print(',');
  Serial.print(adc_volts,5); Serial.print(',');
  Serial.println(xtal_temp);
}

//***********GPS FUNCTIONS*******************************

void print_sys_time(time_t t){
  Serial.print("Sys Time: ");
  int Y = year(t)-30;Serial.print(Y);Serial.print("-");
  int M = month(t); if (M<10) Serial.print("0"); Serial.print(M); Serial.print("-");
  int D = day(t); if (D<10) Serial.print("0"); Serial.print(D); Serial.print("T");
  int h = hour(t);   if (h<10) Serial.print("0"); Serial.print(h); Serial.print(":");  
  int m = minute(t); if (m<10) Serial.print("0"); Serial.print(m); Serial.print(":");  
  int s = second(t); if (s<10) Serial.print("0"); Serial.print(s); Serial.println("Z"); 
  //uint32_t ms = (millis() - syncMillis) % timeSinceSync; 
  //if (ms < 10) Serial.print("00"); else if (ms > 9 && ms < 100) Serial.print("0"); Serial.print(ms);
  //Serial.println("Z");
}

void print_gps_time(Adafruit_GPS &gps){
  Serial.print("GPS Time: ");
  int Y = gps.year; Serial.print("20");Serial.print(Y);Serial.print("-");
  int M = gps.month; if (M<10) Serial.print("0"); Serial.print(M); Serial.print("-");
  int D = gps.day; if (D<10) Serial.print("0"); Serial.print(D); Serial.print("T");
  int h = gps.hour;    if (h<10) Serial.print("0"); Serial.print(h); Serial.print(":");  
  int m = gps.minute;  if (m<10) Serial.print("0"); Serial.print(m); Serial.print(":");  
  int s = gps.seconds; if (s<10) Serial.print("0"); Serial.print(s); Serial.print(".");
  uint16_t ms = gps.milliseconds; 
  if (ms < 10) Serial.print("00"); else if (ms > 9 && ms < 100) Serial.print("0"); Serial.print(ms);
  Serial.println("Z");
}

void set_gps_time(Adafruit_GPS &gps){
  //print_gps_time(gps);
  setTime(gps.hour,gps.minute,gps.seconds,gps.day,gps.month,(gps.year+30));// copy GPS time to system time
  adjustTime(1); // 1pps signal = start of next second
  //syncTime = now(); // remember time of this sync
  synced = true;
}
//***********END GPS FUNCTIONS*******************************

//**********STATE FUNCTIONS**********************************************
void SetStateStandby(){
  tx_stop = millis();
  si5351.output_enable(SI5351_CLK0, 0);
  tx_duration = (tx_stop - tx_start);
  Serial.print(" TX Duration [ms]: "); Serial.println(tx_duration); 
  Serial.print("TX Duration [sec]: "); Serial.println(tx_duration / 1000.0, 3); 
  tx_state = false; //stop transmitting
  synced = false;
  // Turn off the output
  digitalWrite(RED_LED, LOW);
  tcDisable(); //starts the timer
  tx_idx=0;
    
}
void SetStateArmed(){
  
}
void SetStateTX(){
  tx_state = true; //Enable TX flag
  digitalWrite(RED_LED, HIGH);
  Serial.println("TX Conditions Met!...Transmitting!");
  print_sys_time(pps_now);
  Serial.print("TX Freq: "); Serial.println(freq);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.set_freq((freq * 100) + (tx_buffer[tx_idx] * tone_spacing), SI5351_CLK0);
  tx_start = millis();
  tcStartCounter(); //starts the timer
  
  
}
//**********END STATE FUNCTIONS**********************************************

//**********WSPR FUNCTIONS**********************************************
//When timer fires, increment the symbol
void transmit(){
  
}

// Loop through the string, transmitting one character at a time.
void encode()
{
  uint8_t i;
  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK0, 1);
  digitalWrite(RED_LED, HIGH);
  Serial.print("TX Freq: "); Serial.println(freq);
  tx_start = millis();
  for(i = 0; i < symbol_count; i++)
  {
      si5351.set_freq((freq * 100) + (tx_buffer[i] * tone_spacing), SI5351_CLK0);
      delay(tone_delay);
      if(tx_state && btn_press){
        //delay(50); // a little more debounce
        btn_press = false; //button was pressed, reset flag
        tx_arm = false;
        digitalWrite(GREEN_LED, LOW);
        //delay(250);
        break;
      }
  }
  tx_stop = millis();
  si5351.output_enable(SI5351_CLK0, 0);
  tx_duration = (tx_stop - tx_start);
  Serial.print(" TX Duration [ms]: "); Serial.println(tx_duration); 
  Serial.print("TX Duration [sec]: "); Serial.println(tx_duration / 1000.0, 3); 
  tx_state = false; //stop transmitting
  synced = false;
  // Turn off the output
  digitalWrite(RED_LED, LOW);
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);
  // Set the proper frequency and timer CTC depending on mode
  jtencode.wspr_encode(call, loc, dbm, tx_buffer);
}
//**********END WSPR FUNCTIONS**********************************************
//************TIMER FUNCTIONS*************************************************
//REFERENCE:  https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {
  //YOUR CODE HERE 
  //Serial.println("Timer: ");// Serial.println((SystemCoreClock / sampleRate - 1));
  tx_idx +=1;
  symbol_step = true;
  // END OF YOUR CODE
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //Writing a 1 to INTFLAG.bit.MC0 clears the interrupt so that it will run again
}

/* 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
 void tcConfigure(int sampleRate)
{
 // Enable GCLK for TCC2 and TC5 (timer counter input clock)
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

 tcReset(); //reset TC5

 // Set Timer counter Mode to 16 bits
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC5 mode as match frequency
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 
 //set prescaler and enable TC5
 //you can use different prescaler divisons here like TC_CTRLA_PRESCALER_DIV1 to get different ranges of frequencies
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE; 
 //TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_ENABLE; 
 
 //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
 TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
 while (tcIsSyncing());
 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}
//************END TIMER FUNCTIONS*************************************************

//***********OTHER FUNCTIONS*******************************
void pps_isr(){
  //SerialUSB.println("THERMO ISR Fired!!!");
  pps_fired = true;
}

void btn_isr(){
  //SerialUSB.println("THERMO ISR Fired!!!");
  //delay(50);
  if(digitalRead(BUTTON) == LOW){
    delay(50);   // delay to debounce
    if (digitalRead(BUTTON) == LOW){
      btn_press = true;
      delay(200); //delay to avoid extra triggers
    }
  }
}
void toggle_led(){
  led_state = ~led_state;
  digitalWrite(RED_LED, led_state);
  //digitalWrite(GREEN_LED, led_state);
}

void GridLocator(char *dst, float latt, float lon) {
  int o1, o2;
  int a1, a2;
  float remainder;
  // longitude
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (float)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  // latitude
  remainder = latt + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (float)a1 * 10.0;
  a2 = (int)(remainder);

  dst[0] = (char)o1 + 'A';
  dst[1] = (char)a1 + 'A';
  dst[2] = (char)o2 + '0';
  dst[3] = (char)a2 + '0';
  dst[4] = (char)0;
}
//***********END OTHER  FUNCTIONS*******************************
