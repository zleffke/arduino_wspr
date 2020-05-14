// based on Adafruit GPS example

#include <Adafruit_GPS.h>           // Adafruit Ultimate GPS Library; https://github.com/adafruit/Adafruit_GPS
#include <TimeLib.h>                // Time functions  https://github.com/PaulStoffregen/Time

//****Custom Board Pinouts*********************************
//OUTPUT
#define RED_LED       13
#define GREEN_LED     8
#define GPS_RESET     A3 //RESET by pulling low
#define GPS_FIX       A4 //Same as GPS LED Drive, when fix pulses high every 15sec, when no fix pulses high every 1 sec
#define GPS_PPS       11 //pulses high, usually low
#define GPS_ENABLE    A5 //Active LOW, pull high to DISABLE GPS

// what's the name of the hardware serial port?
#define GPSSerial Serial1
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

//*****Time Variables********
#define SYNC_INTERVAL          10 // time, in seconds, between GPS sync attempts
#define SYNC_TIMEOUT           60 // time(sec) without GPS input before error
time_t syncTime = 0; // time of last GPS or RTC synchronization
time_t  pps_now = 0; //'now' time when PPS fires
unsigned long syncMillis = 0; // millis() at sync
unsigned long timeSinceSync = 0;
bool synced     = false; //Time synchronized yet?


uint32_t      timer      = millis();
uint8_t       led_state  = 0;
volatile bool pps_fired  = false;


void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");


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

  //Setup interrupts
  int irq_pps = digitalPinToInterrupt(GPS_PPS);
  attachInterrupt(irq_pps, pps_isr, RISING);

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
}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  SyncCheck();    
  if (pps_fired){
    pps_now = now();
    Serial.println("PPS FIRED!");
    pps_fired = false; //reset PPS flag
    toggle_led();
    //SyncCheck();    
    //print_gps(GPS);
    print_gps_time(GPS);
    print_sys_time(pps_now);
  }
}

void SyncCheck()                                        // called from void loop
// Manage synchronization of clock to GPS module
// First, check to see if it is time to synchronize
// Do time synchronization on the 1pps signal
// This call must be made frequently (keep in main loop)
{
  timeSinceSync = now()-syncTime;// how long has it been since last sync?
  if (timeSinceSync >= SYNC_INTERVAL){// is it time to sync with GPS yet?
    SyncWithGPS();// yes, so attempt it.
  }
}

void SyncWithGPS(){
  Serial.println("SYNCING TO GPS");
  setTime(GPS.hour,GPS.minute,GPS.seconds,GPS.day,GPS.month,(GPS.year+30));// copy GPS time to system time
  adjustTime(1); // 1pps signal = start of next second
  syncTime = now(); // remember time of this sync
  syncMillis = millis();
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

void pps_isr(){
  //SerialUSB.println("THERMO ISR Fired!!!");
  pps_fired = true;
}

void toggle_led(){
  led_state = ~led_state;
  digitalWrite(RED_LED, led_state);
  digitalWrite(GREEN_LED, led_state);
  
  
}
