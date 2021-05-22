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
volatile uint32_t pulseCount = 0;

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
  //analogReadResolution(12);

  //Setup interrupts
  //int irq_pps = digitalPinToInterrupt(GPS_PPS);
  //attachInterrupt(irq_pps, pps_isr, RISING);
  int irq_btn = digitalPinToInterrupt(BUTTON);
  attachInterrupt(irq_btn, btn_isr, FALLING);
  int irq_clk = digitalPinToInterrupt(CLK1_PIN);
  attachInterrupt(irq_clk, clk_isr, RISING);

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
  //TimerConfigure();
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

void clk_isr(){
  pulseCount++;
  if (pulseCount > 1e6){
    digitalWrite(TX_LED1, toggle);
    toggle = ~toggle;
    pulseCount = 0;
  }
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
  float adc_volts = adc_val / 1024 * 3.3 + adc_cal_val;
  xtal_temp = 100.0 * adc_volts - 50.0; //C
  //xtal_temp = (adc_volts - V0)/Tc;
  
  
  //val * 0.0625;
  //Serial.print(adc_val); Serial.print(',');
  //Serial.print(adc_volts,5); Serial.print(',');
  //Serial.println(xtal_temp);
}


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
