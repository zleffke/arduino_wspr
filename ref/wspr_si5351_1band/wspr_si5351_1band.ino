#include <si5351.h>
#include <JTEncode.h>
#include <rs_common.h>
#include <int.h>
#include <string.h>
#include <Adafruit_GPS.h> // Adafruit Ultimate GPS Library; https://github.com/adafruit/Adafruit_GPS
#include <TimeLib.h>      // Time functions  https://github.com/PaulStoffregen/Time
#include "Wire.h"

//****Board Pinouts*********************************
//OUTPUT
#define RED_LED       13
#define GREEN_LED     8
#define GPS_RESET     A3 //RESET by pulling low
#define GPS_FIX       A4 //Same as GPS LED Drive, when fix pulses high every 15sec, when no fix pulses high every 1 sec
#define GPS_PPS       11 //pulses high, usually low
#define GPS_ENABLE    A5 //Active LOW, pull high to DISABLE GPS
#define BUTTON                  12

// ****WSPR Details********************
#define WSPR_TONE_SPACING       146          // ~1.46 Hz
//#define WSPR_DELAY              683          // Delay value for WSPR
#define WSPR_DELAY              681          // slightly less to account for if statement in TX state
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
uint8_t dbm = 11;
uint8_t tx_buffer[255];

uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;
//*****Time Variables********
#define SYNC_INTERVAL          10 // time, in seconds, between GPS sync attempts
#define SYNC_TIMEOUT           60 // time(sec) without GPS input before error
time_t syncTime = 0; // time of last GPS or RTC synchronization
time_t  pps_now = 0; //'now' time when PPS fires
time_t gps_time = 0; //Set GPS Time
unsigned long syncMillis = 0; // millis() at sync
unsigned long timeSinceSync = 0;
bool synced     = false; //Time synchronized yet?
bool gps_lock = false;
uint32_t tx_start = 0;
uint32_t tx_stop  = 0;
uint32_t tx_duration = 0;


uint32_t      timer      = millis();
uint8_t       led_state  = 0;
bool tx_state = false;
bool resync = false;
uint8_t tx_arm = 0;

uint8_t STATE = 0; //0 = STANDBY, 1=GPS_SYNCED, 2=ARMED, 3=TX,

volatile bool btn_press  = false; //set in button ISR
volatile bool pps_fired  = false; //set in PPS ISR


void setup()
{
  Serial.begin(115200);
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz

  // initialize digital pin outputs.
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
  delay(3000);

  //freq = WSPR_DEFAULT_FREQ;
  //freq = wspr_band.b40;
  symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
  tone_spacing = WSPR_TONE_SPACING;
  tone_delay = WSPR_DELAY;

  // Encode the message in the transmit buffer
  // This is RAM intensive and should be done separately from other subroutines
  set_tx_buffer();
}

void loop(){
  
  if (btn_press){ //ARM For TX on next time interval.  Every two minutes.
    btn_press = false;
    tx_arm = ~tx_arm;
    Serial.println(tx_arm);
    digitalWrite(GREEN_LED, tx_arm);
    //toggle_led();
    //encode();
  }
  

  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }  
  if (pps_fired){
    pps_fired = false; //reset PPS flag
    pps_now = now();
    //Serial.println("\nPPS FIRED!");
    //Serial.print("Time Synced to GPS: "); Serial.println(synced);
    //toggle_led();
    SyncCheck(pps_now);    
    //print_gps(GPS);
    print_gps_time(GPS);
    print_sys_time(pps_now);
    if (tx_arm){
      uint8_t s = second(pps_now);
      uint8_t m = minute(pps_now);
      //Serial.print("TX ARMED: "); Serial.println(tx_arm);
      //Serial.print(" Seconds: "); Serial.println(s);
      //Serial.print(" Minutes: "); Serial.print(m); Serial.print(" "); Serial.println(m%10);
      
      if (synced && s==1 && m%2==0){
        
        if      (m%10==0){freq=wspr_band.b40;}
        else if (m%10==2){freq=wspr_band.b30;}
        else if (m%10==4){freq=wspr_band.b40;}
        else if (m%10==6){freq=wspr_band.b30;}
        else if (m%10==8){freq=wspr_band.b20;}
        
        //print_gps_time(GPS);
        print_sys_time(pps_now);
        Serial.println("TX Conditions Met!...Transmitting!");
        tx_state = true; //Enable TX flag
        encode();//Start transmitting
      }
    }
  }
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

void SyncCheck(time_t &time_now)                                        // called from void loop
// Manage synchronization of clock to GPS module
// First, check to see if it is time to synchronize
// Do time synchronization on the 1pps signal
// This call must be made frequently (keep in main loop)
{
  if (GPS.fix){
    //print_gps_time(GPS);
    //print_sys_time(pps_now);
    gps_lock = true;
    //Serial.print("GPS Seconds: "); Serial.println(GPS.seconds);
    //Serial.print("SYS Seconds: "); Serial.println(second(time_now));
    if (second(time_now) == 0){
      if (GPS.seconds ==59){synced = true;}
      else{synced = false;}
    }
    else{synced = GPS.seconds<second(time_now);}
    //Serial.print("GPS Seconds: "); Serial.println(GPS.seconds);
    //Serial.print("SYS Seconds: "); Serial.println(second(now()));
    timeSinceSync = now()-syncTime;// how long has it been since last sync?
    if ((timeSinceSync >= SYNC_INTERVAL) | (!synced)){// is it time to sync with GPS yet?
      SyncWithGPS();// yes, so attempt it.
    }
  }
  else{
    gps_lock = false;
    synced = false;
  }
}

void SyncWithGPS(){
  Serial.println("\nSYNCING TO GPS");
  setTime(GPS.hour,GPS.minute,GPS.seconds,GPS.day,GPS.month,(GPS.year+30));// copy GPS time to system time
  adjustTime(1); // 1pps signal = start of next second
  syncTime = now(); // remember time of this sync
  //Serial.print("GPS Seconds: "); Serial.println(GPS.seconds);
  //Serial.print("SYS Seconds: "); Serial.println(second(syncTime));
  //synced = GPS.seconds<second(syncTime);
  //syncMillis = millis();
}

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

void print_gps_time(Adafruit_GPS &GPS){
  Serial.print("GPS Time: ");
  int Y = GPS.year; Serial.print("20");Serial.print(Y);Serial.print("-");
  int M = GPS.month; if (M<10) Serial.print("0"); Serial.print(M); Serial.print("-");
  int D = GPS.day; if (D<10) Serial.print("0"); Serial.print(D); Serial.print("T");
  int h = GPS.hour;    if (h<10) Serial.print("0"); Serial.print(h); Serial.print(":");  
  int m = GPS.minute;  if (m<10) Serial.print("0"); Serial.print(m); Serial.print(":");  
  int s = GPS.seconds; if (s<10) Serial.print("0"); Serial.print(s); Serial.print(".");
  uint16_t ms = GPS.milliseconds; 
  if (ms < 10) Serial.print("00"); else if (ms > 9 && ms < 100) Serial.print("0"); Serial.print(ms);
  Serial.println("Z");
}


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
      delay(50); //delay to avoid extra triggers
    }
  }
}
void toggle_led(){
  led_state = ~led_state;
  //digitalWrite(RED_LED, led_state);
  digitalWrite(GREEN_LED, led_state);
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

void print_gps(Adafruit_GPS &GPS){
  Serial.print("Time: ");
  if (GPS.hour < 10) { Serial.print('0'); }
  Serial.print(GPS.hour, DEC); Serial.print(':');
  if (GPS.minute < 10) { Serial.print('0'); }
  Serial.print(GPS.minute, DEC); Serial.print(':');
  if (GPS.seconds < 10) { Serial.print('0'); }
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  if (GPS.milliseconds < 10) {
    Serial.print("00");
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    Serial.print("0");
  }
  Serial.println(GPS.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("latitude: ");
    Serial.println(GPS.latitudeDegrees, 8); 
    Serial.print("longitude: ");
    Serial.println(GPS.longitudeDegrees, 8); 
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    Serial.println();
  }
}
