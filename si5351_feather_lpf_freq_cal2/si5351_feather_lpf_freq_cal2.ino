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
//#define GPS_RESET     A3 //RESET by pulling low
//#define GPS_FIX       A4 //Same as GPS LED Drive, when fix pulses high every 15sec, when no fix pulses high every 1 sec
#define GPS_PPS       11 //pulses high, usually low
//#define GPS_ENABLE    A5 //Active LOW, pull high to DISABLE GPS
#define BUTTON        12
#define XTAL_TEMP     A1
#define LPF_S0        9
#define LPF_S1        10
#define CLK1_PIN      6  //Clock 1 feedback....future use for GPS disciplining

#define WSPR_DEFAULT_FREQ       14097200UL
//#define FREQ_CORRECTION         15575UL
#define FREQ_CORRECTION         0UL
#define REF_FREQ                1000000UL

uint32_t freq = WSPR_DEFAULT_FREQ;
uint8_t band_idx = 0;

// Class instantiation
Si5351 si5351;
float   xtal_temp = 0;
uint8_t led_state  = 0;
uint8_t tx_arm = 0;
bool    tx_state = false;
bool    next_pps_tx = false;
int serByte = 0; 
String inString = "";    // string to hold input

volatile bool btn_press  = false; //set in button ISR
volatile bool pps_fired  = false; //set in PPS ISR

// Setup TC3 to capture pulse-width and period
volatile boolean periodComplete;
volatile uint16_t isrPeriod;
volatile uint16_t isrPulsewidth;
uint16_t period;
uint16_t pulsewidth;
uint8_t toggle = 0;

void setup()
{
  Serial.begin(115200);
  
  // initialize digital pin outputs.
  pinMode(TX_LED1, OUTPUT);
  digitalWrite(TX_LED1, LOW);
  pinMode(TX_LED2, OUTPUT);
  digitalWrite(TX_LED2, LOW);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  //pinMode(GPS_RESET, OUTPUT);
  //digitalWrite(GPS_RESET, HIGH);
  //pinMode(GPS_ENABLE, OUTPUT);
  //digitalWrite(GPS_ENABLE, LOW); //ENABLE GPS
  pinMode(LPF_S0, OUTPUT);
  digitalWrite(LPF_S0, LOW);
  pinMode(LPF_S1, OUTPUT);
  digitalWrite(LPF_S1, LOW);
  //initialize digital pin inputs
  pinMode(CLK1_PIN, INPUT);
  //pinMode(GPS_FIX, INPUT);
  pinMode(GPS_PPS, INPUT);
  //pinMode(BUTTON, INPUT_PULLUP); // Use a button connected to pin 12 as a transmit trigger
  analogReadResolution(12);

  //Setup interrupts
  //int irq_pps = digitalPinToInterrupt(GPS_PPS);
  //attachInterrupt(irq_pps, pps_isr, RISING);
  //int irq_btn = digitalPinToInterrupt(BUTTON);
  //attachInterrupt(irq_btn, btn_isr, FALLING);

  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, FREQ_CORRECTION);
  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially

  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.set_freq(REF_FREQ*100, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1); // Turn on the REFERENCE 

  /*
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
  */

  //Setup Timer and start
  //tcConfigure(WSPR_BAUD); //configure the timer to run at <sampleRate>Hertz
  //tcStartCounter(); //starts the timer
  //tcDisable();
  delay(2000);
  TimerConfigure();
}


void loop(){
  while (Serial.available() > 0) {
    int inChar = Serial.read();

    if (inChar != '\n'){
      inString += (char)inChar;
    }
    else if (inChar == '\n') {
      processSerialCommand(inString);
    }
  }
  if (btn_press){
    btn_press = false;
    Serial.println("Button Pressed!");
    if (tx_state){
      SetStateDisabled();
    }
    else{
      SetStateEnabled();
    }
  }
  get_xtal_temp();
  //si5351.set_freq((freq * 100)), SI5351_CLK0);
  inString = ""; //reset input string
  //Serial.println(TC4->COUNT32.COUNT.reg);
  
}


void processSerialCommand(String inString){
  Serial.println(inString);
  if (inString[0] == 't'){ //print temp
    Serial.println(xtal_temp);
  }
  else if (inString[0] == 'f'){
    String f_str = inString.substring(inString.indexOf(',')+1);
    int idx = f_str.indexOf('e');
    if(idx>0){
      float m = f_str.substring(0,idx).toFloat();
      float n = f_str.substring(idx+1).toFloat();
      freq = (uint32_t)(m* pow(10,n));
      si5351.set_freq(freq * 100,SI5351_CLK0);
      Serial.print("Set Freq [MHz]: "); Serial.println(freq / 1e6);
    }
  }
  else if (inString[0] == 'c'){
    String f_str = inString.substring(inString.indexOf(',')+1);
    uint32_t corr = (uint32_t)strtoul(f_str.c_str(), NULL, 10);
    Serial.print("Setting Correction: "); Serial.println(corr);
    si5351.set_correction(corr, SI5351_PLL_INPUT_XO);
  }

  else if (inString[0] == 'e'){
    SetStateEnabled();
  }
  else if (inString[0] == 'd'){
    SetStateDisabled();
  }
  else if (inString[0] == 'q'){
    Serial.println(freq);
  }
  else if (inString[0] == 'g'){
    uint32_t corr = si5351.get_correction(SI5351_PLL_INPUT_XO);
    Serial.println(corr);
  }
  
//si5351_pll_input
  
}



//---- Input Capture Timer---------
void TC4_Handler()
{
    uint32_t count ;
  
    // Check for match counter 0 (MC0) interrupt
    // read automatically clears flag      
    if (TC4->COUNT16.INTFLAG.bit.MC0)
    {
        REG_TC4_READREQ = TC_READREQ_RREQ | TC_READREQ_ADDR(0x18);
        while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
        count = REG_TC4_COUNT32_CC0;

        // print out for debug
        Serial.print("Count capture: ") ;
        Serial.print(count) ;
        Serial.print("\r\n") ;
    }
  
    static bool toggle = false ;
    toggle = !toggle ;
    digitalWrite(TX_LED2, toggle) ;
}





void TimerConfigure(){
  //ref:  https://forum.arduino.cc/index.php?topic=515358.0
  // change here to use main clock or input pin
  
  bool use_clk_pin = false ;

  Serial.print("\r\nBegin config\r\n");

  //pinMode(11, OUTPUT) ;
  //pinMode(12, OUTPUT) ;

  // setup main clocks first
  REG_PM_APBAMASK |= PM_APBAMASK_GCLK ;
  REG_PM_APBBMASK |= PM_APBBMASK_PORT ;
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 ;

  //
  //    Setup gate input on PA17 (M0 pin 13) <--ORIGINAL
  //    GPS = D11 = PA16 <-NEW PIN INPUT, GPS PPS

  // send generic GCLK0 to the EIC peripheral
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_EIC ;
  while(GCLK->STATUS.bit.SYNCBUSY) ;

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_EVSYS_0 ;
  while(GCLK->STATUS.bit.SYNCBUSY) ;

  // Send pin PA17 to EIC (line 1)
  //PORT->Group[PORTA].PMUX[17 >> 1].reg |= PORT_PMUX_PMUXO_A ;
  //PORT->Group[PORTA].PINCFG[17].reg |= PORT_PINCFG_PMUXEN ;
  // Send pin PA16 to EIC (line 1)
  PORT->Group[PORTA].PMUX[16 >> 1].reg |= PORT_PMUX_PMUXE_A ;
  PORT->Group[PORTA].PINCFG[16].reg |= PORT_PINCFG_PMUXEN ;


  // enable ext1 int rising edge to event system
  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO1 ;
  REG_EIC_CONFIG0 |= EIC_CONFIG_SENSE1_RISE ;
  REG_EIC_CTRL |= EIC_CTRL_ENABLE;
  while (EIC->STATUS.bit.SYNCBUSY);

  // direct ext3 event to TC4
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU) ;
  //REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS
  //                    | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_1) | EVSYS_CHANNEL_CHANNEL(0) ;
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_RISING_EDGE | EVSYS_CHANNEL_PATH_RESYNCHRONIZED
                      | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_1) | EVSYS_CHANNEL_CHANNEL(0) ;

  //
  //    Setup timer clock input on PA21 (pin 7 on M0)
  //    CLK1_PIN = D6 = PA20

  // Source GCLK from PA21 (gclkio 5)
  PORT->Group[PORTA].PMUX[20 / 2].reg = PORT_PMUX_PMUXE_H ;
  PORT->Group[PORTA].PINCFG[20].reg = PORT_PINCFG_PMUXEN ;

  // tick count: gclk_io5 => tc4-5
  REG_GCLK_GENDIV = GCLK_GENDIV_ID(5) ; // GCLK5: no div

  // clock from pin PA21
  //REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_SRC_GCLKIN  | GCLK_GENCTRL_ID(5) ;
  //while (GCLK->STATUS.bit.SYNCBUSY);



  if(use_clk_pin) {
      // Feed from GCLK_IO
      REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_SRC_GCLKIN  | GCLK_GENCTRL_ID(5) ;
      while (GCLK->STATUS.bit.SYNCBUSY);
  } else {
      // clock from main clock
      REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(5) ;
      while (GCLK->STATUS.bit.SYNCBUSY);
  }

  // Feed GCLK5 to TC4/TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK5 | GCLK_CLKCTRL_ID_TC4_TC5 ;
  while (GCLK->STATUS.bit.SYNCBUSY);
  
  //
  // general timer TC4-TC5 setup for 32 bit
  //

  // Enable event input
  REG_TC4_EVCTRL |= TC_EVCTRL_TCEI ;

  // set capture on channel 0
  REG_TC4_READREQ = TC_READREQ_RREQ | TC_READREQ_ADDR(0x06) ;
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY) ;
  REG_TC4_CTRLC |= TC_CTRLC_CPTEN0 ;
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY) ;

  // enable timer TC4 interrupt for Capture events
  NVIC_SetPriority(TC4_IRQn, 0) ;
  NVIC_EnableIRQ(TC4_IRQn) ;
  REG_TC4_INTENSET = TC_INTENSET_MC0 ;

  // no prescaler, 32 bit mode: run timer!
  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_MODE_COUNT32 | TC_CTRLA_ENABLE ;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY) ;

  Serial.print("config done\r\n") ;
}



void SetStateEnabled(){
  si5351.output_enable(SI5351_CLK0, 1);
  Serial.println("Clock 0 Output Enabled");
  digitalWrite(TX_LED1, HIGH);
  tx_state = true;
}
void SetStateDisabled(){
  si5351.output_enable(SI5351_CLK0, 0);
  Serial.println("Clock 0 Output Disabled");
  digitalWrite(TX_LED1, LOW);
  tx_state = false;
}



float get_xtal_temp()
{
  float adc_cal_val = -0.035;
  float adc_val = analogRead(XTAL_TEMP);
  float adc_volts = adc_val / 4096 * 3.3 + adc_cal_val;
  xtal_temp = 100.0 * adc_volts - 50.0; //C
  //xtal_temp = (adc_volts - V0)/Tc;
  
  
  //val * 0.0625;
  //Serial.print(adc_val); Serial.print(',');
  //Serial.print(adc_volts,5); Serial.print(',');
  //Serial.println(xtal_temp);
}

//************TIMER FUNCTIONS*************************************************
//REFERENCE:  https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {
  //YOUR CODE HERE 
  //Serial.println("Timer: ");// Serial.println((SystemCoreClock / sampleRate - 1));
  //tx_idx +=1;
 // symbol_step = true;
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
