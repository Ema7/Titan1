/*
 TITAN HAB flight computer software
 Modifications copyright 2015 by Emanuel Bombasaro

 The original code and subfiles is by:
   HABDuino Tracker
   http://www.habduino.org
   (c) Anthony Stirk M0UPU
   November 2014 Version 3.0.3
   Please visite for lates release and details: https://github.com/HABduino/HABduino

   Modifications done are:
   Make it work with Arduino Mega 2560 rev3
   All parts regarding additional sensors and logging to SD-card.
   Some modifications on formatting and variable naming.
   However, please start with looking into the original code first!

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 See <http://www.gnu.org/licenses/>.

 This code is suitable for the Arduino MEGA 2560 rev3.
 Pin connection as outlined below.

 HABduino, mounat as it is.
 NB. I2C is connected as follows:
   connect SCL to analog 21
   connect SDA to analog 20
 thus the pins on the HABduino are not correct.

 SD card attached to SPI bus as follows:
   MISO - pin 50
   MOSI - pin 51
   SLK - pin 52
   SS - pin 53

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
created 25-XII-2014 EMBO
edited  20- II-2015 EMBO
edited  03- III-2015 EMBO fixed sd write, no in the delay loop seems to work better.
edited  16- III-2015 EMBO reduced error value for sensors
edited  18- III-2015 EMBO gyro, acc and mag calibration ands sensor redadings
edited  24- III-2015 EMBO mag calibration now advance
*/

// include ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//HABduino
#include <util/crc16.h> // crc check sum
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h> // watch dog
#include <Wire.h> // including I2C
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include "ax25modem.h"
// sensors
#include "Sensor.h"      // info on sensors, variables, strucutres
#include "TSL2561.h"     // light sensor
#include "HTU21DF.h"     // temp & humidity
#include "MPL3115A2.h"   // altimeter
#include "MCP9808.h"     // temperrature precision
#include "BMP085_U.h"    // pressure sensor IMU
// IMU 10 DOF
#include "L3GD20.h"      // gyro sensors
#include "LSM303_U.h"    // acc and mag sensor
/*
coordinates as indicated on sensor
  gyro x, y, z rotation deg/s
  acc  x, y, z accelaration m/s^2 (negative in direction of axis)
  mag  x, y, z gauss
  --------------------------------
  roll postive x axis
  pitch positive y axis
  heading in positiv x direction
*/
// sd card
#include <SPI.h>
#include <SD.h>

// configurations +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// define constants
#define ONE_SECOND F_CPU / 1024 / 16  // define one second
#define MTX2_FREQ 434.485             // format 434.XXX
#define POWERSAVING                   // GPS power saving mode

// some help constants
#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (50)
#define REST_BYTES     (5)

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)

// RTTY HABduino
#define ASCII 7          // ASCII 7 or 8 bit
#define STOPBITS 2       // Either 1 or 2 stop bits
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 50     // RTTY Baud rate (Recommended = 50)
#define MTX2_SHIFT 425   // shift rate
#define MTX2_OFFSET 0    // 0-100 Slightly adjusts the frequency by increasing the PWM

// pins ###################################################################
#define LED_WARN 12      // warning LED, red (hardware set)
#define LED_OK 13        // ok LED, green (hardware set)

#define BATTERY_ADC A0   // battery voltage check 0-1023 coresponding to 0-5V (hardware set)
#define GPS_ON 2         // GPS power on (hardware set)
#define ONE_WIRE_BUS 5   // BUS pin for temperature sensor on HABduino (hardware set)

#define HX1_ENABLE 6     // for addtional ATTY element
#define HX1_TXD 3        // TXD for addtional ATTY (hardware set)
#define MTX2_ENABLE 7    // MTX2 enable  (hardware set)
#define MTX2_TXD 11      // MTX2 TXD (hardware set)

// ########################################################################
// call sign
char callsign[9] = "TITAN_1";          // MAX 9 CHARACTERS!!
// limit of staying in pedestrian mode
int pedlim=2000; // meters
// logfile name
char logfilename[13] = "01datlog.dat";    // cmax 8 char befor dot!

// define errorstatus
int errorstatus=0;
/* Error Status Bit Level Field :
 Bit 0 = GPS Error Condition Noted Switch to Max Performance Mode
 Bit 1 = GPS Error Condition Noted Cold Boot GPS
 Bit 2 = DS18B20 temp sensor status 0 = OK 1 = Fault
 Bit 3 = Current Dynamic Model 0 = Flight 1 = Pedestrian
 Bit 4 = PSM Status 0 = PSM On 1 = PSM Off
 Bit 5 = Lock 0 = GPS Locked 1= Not Locked

 Bit 6 = TSL2561 lumosity sensor status 0 = OK 1 = Fault
 Bit 7 = HTU21DF temp humidity sensor status 0 = OK 1 = Fault
 Bit 8 = MPL3115A2 pressure sensor status 0 = OK 1 = Fault
 Bit 9 = MCP9808 temperatur precision sensor status 0 = OK 1 = Fault
 Bit 10= BMP085 pressure sensor status 0 = OK 1 = Fault
 Bit 11= Gyro sensor status 0 = OK 1 = Fault
 Bit 12= Acc sensor status 0 = OK 1 = Fault
 Bit 13= Mag sensor status 0 = OK 1 = Fault
 Bit 14= logging status 0 = OK 1 = Fault

 So error 8 means the everything is fine just the GPS is in pedestrian mode.
 Below pedlim meters the code puts the GPS in the more accurate pedestrian mode.
 Above pedlim meters it switches to dynamic model 6 i.e flight mode and turns the LED's off for additional power saving.
 So as an example error code 40 = 1010 0000 0000 means GPS not locked and in pedestrian mode.
 */

// define vaiables
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
volatile int count=1;
volatile boolean lockvariables = 0;
volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;

char txstring[100]; // string values are stored in for transmission
char tmpS[2]; // temporay string for hour, minute and second logging
uint8_t buf[60];
uint8_t lock =0, sats = 0, hour = 0, minute = 0, second = 0;
uint8_t oldhour = 0, oldminute = 0, oldsecond = 0;
int GPSerror = 0,navmode = 0,psm_status = 0,lat_int=0,lon_int=0,temperature1=0;
int32_t lat = 0, lon = 0, alt = 0, maxalt = 0, lat_dec = 0, lon_dec =0 ,tslf=0;
unsigned long currentMillis;
long previousMillis = 0;
int batteryadc_v,battvaverage=0,aprstxstatus=0;
int32_t battvsmooth[5] ;
int aprs_tx_status = 0, aprs_attempts = 0;
unsigned long startTime;
char comment[3]={' ', ' ', '\0'};
static const uint8_t PROGMEM _sine_table[] = {
#include "sine_table.h"
}; // sinus wave
// sensor variables
uint32_t lum1=0, lux1=0;
uint16_t ir1=0, full1=0;
float temperature3=0, temperature4=0, humidity1=0, pressure1=0, alt1=0, temperature5=0, pressure2=0, alt2=0, temperature6=0;
float gyroX=0, gyroY=0, gyroZ=0, accX=0, accY=0, accZ=0, magX=0, magY=0, magZ=0, roll=0, pitch=0, head=0;
float accXnorm, accYnorm, magXnorm, magYnorm, magZnorm, magXcomp, magYcomp;
File dataFile; // log data file

// start serials and sensors
SoftwareSerial MTX2_EN(LED_WARN, MTX2_ENABLE); // RX, TX serial to comunitate with the MTX2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
TSL2561 tsl(TSL2561_ADDR_FLOAT); // the ADDR pin float (addr 0x39), light sensor
HTU21DF htu = HTU21DF();  // temp humidity senosor
MPL3115A2 baro = MPL3115A2();
MCP9808 pretempsen = MCP9808(); // Create the MCP9808 temperature sensor object
BMP085_Unified bmp = BMP085_Unified(10085); // pressuer sensor IMU
L3GD20 gyro; // start gyro
LSM303_Accel_Unified accel = LSM303_Accel_Unified(54321); // start acc sensor
LSM303_Mag_Unified mag = LSM303_Mag_Unified(12345); // start mag sensor

// calib gyro offset 0 is no offeset
const float gyroXoff=-2.54, gyroYoff=0.36, gyroZoff=-0.33;
// calibrate mag
const float magEC[3] = {10.1555, -0.271402, -26.0203};
const float magET[3][3] = {{0.965256, -0.00477968, 0.0596485}, {-0.00477968, 0.932893, 0.00230931}, {0.0596485, 0.00230931, 0.897073}};
//
const int chipSelect = 53;  // sd card chip select (CS) pin, on Mega 2560 pin 53

// main setup function ++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup()
{
wdt_enable (WDTO_8S); // watchdog .........................................
  /* rest times
    WDTO_15MS
    WDTO_30MS
    WDTO_60MS
    WDTO_120MS
    WDTO_250MS
    WDTO_500MS
    WDTO_1S
    WDTO_2S
    WDTO_4S
    WDTO_8S
 */
  // set pin modes
  pinMode(MTX2_TXD, OUTPUT);
  pinMode(LED_WARN, OUTPUT);
  pinMode(HX1_ENABLE, OUTPUT);
  pinMode(LED_OK,OUTPUT);
  pinMode(MTX2_ENABLE, OUTPUT);
  pinMode(GPS_ON, OUTPUT);
  pinMode(BATTERY_ADC, INPUT);

  // setups
  blinkled(6);
  setMTX2Frequency();
  Serial.begin(9600);
  digitalWrite(MTX2_ENABLE,HIGH);
  blinkled(5);
  digitalWrite(GPS_ON,HIGH);
  blinkled(4);
  resetGPS();
  blinkled(3);
  // set timer for TXD
  TCCR1B = TCCR1B & 0b11111000 | 0x01; // Sets fast PWM on pin 11 12 controled by timer1
#ifdef APRS
  ax25_init(); // initialize APRS
#endif
  blinkled(2);
  setupGPS();
  blinkled(1);
  initialise_interrupt();
  // initialise sensors
  sensors.begin(); // begin temp sensor
  //tempsensors=sensors.getDeviceCount(); // we know its only 1
  // ligth sensor settings and check if senosrs are on
  // Set or unset bit 6 indicating the light sensor is faulty
  if(!tsl.begin()) {
    tsl.setGain(TSL2561_GAIN_0X); // set no gain (for bright situtations)
    tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS); // medium resolution and speed
    errorstatus |=(1 << 6);}
  else {errorstatus &= ~(1 << 6);}
  // Set or unset bit 7 indicating the temperature and humidity sensor is faulty
  if(!htu.begin()) {errorstatus |=(1 << 7);} else {errorstatus &= ~(1 << 7);}
  // Set or unset bit 8 indicating the altimeter and presure sensor is faulty
  if(!baro.begin()) {errorstatus |=(1 << 8);} else {errorstatus &= ~(1 << 8);}
  // Set or unset bit 9 indicating the precision temperature sensor is faulty
  if(!pretempsen.begin()) {errorstatus |=(1 << 9);} else {errorstatus &= ~(1 << 9);}
  // Set or unset bit 10 indicating the pressuer IMU sensor is faulty
  if(!bmp.begin()) {errorstatus |=(1 << 10);} else {errorstatus &= ~(1 << 10);}
  // Set or unset bit 11 indicating the gyro IMU sensor is faulty
  if (!gyro.begin(gyro.L3DS20_RANGE_500DPS)) {errorstatus |=(1 << 11);} else {errorstatus &= ~(1 << 11);}
  // Set or unset bit 12 indicating the accel IMU sensor is faulty
  if (!accel.begin()) {errorstatus |=(1 << 12);} else {errorstatus &= ~(1 << 12);}
  // Set or unset bit 13 indicating the mag IMU sensor is faulty
  mag.enableAutoRange(true); // Enable auto-gain mag sensor
  if (!mag.begin()) {errorstatus |=(1 << 13);} else {errorstatus &= ~(1 << 13);}
  // sd card
  pinMode(chipSelect, OUTPUT);
  // see if the card is present and can be initialized else writ erro in bit 13
  if (!SD.begin(chipSelect)){errorstatus |=(1 << 14);} else {
    // open (create) file and close it, as check
    dataFile = SD.open(logfilename, FILE_WRITE);
    dataFile.close();
    errorstatus &= ~(1 << 14);
    }
wdt_reset(); // ...........................................................
}
// end setup function

// begin main loop ########################################################
void loop()   {
wdt_reset(); // ...........................................................
  oldhour=hour;
  oldminute=minute;
  oldsecond=second;
  gps_check_nav();
  // Set bit 5 (Lock 0 = GPS Locked 1= Not Locked)
  if(lock!=3) {errorstatus |=(1 << 5);}else{errorstatus &= ~(1 << 5);}
  checkDynamicModel();
wdt_reset(); // ...........................................................
#ifdef APRS
  if(sats>=4){
    if (aprs_tx_status==0)
    {
      startTime=millis();
      aprs_tx_status=1;
    }
    if(millis() - startTime > (APRS_TX_INTERVAL*60000)) {
      aprs_tx_status=0;
      send_APRS();
      aprs_attempts++;
    }
  }
#endif
wdt_reset(); // ...........................................................
#ifdef POWERSAVING
  if((lock==3) && (psm_status==0) && (sats>=5) &&((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0)) // Check we aren't in an error condition
  {
    setGPS_PowerSaveMode();
    wait(1000);
    psm_status=1;
    errorstatus &= ~(1 << 4); // Set Bit 4 Indicating PSM is on
  }
#endif
  if(!lockvariables) {

    prepare_data();
    if(alt>maxalt && sats >= 4)
    {
      maxalt=alt;
    }
  }
  if((oldhour==hour&&oldminute==minute&&oldsecond==second)||sats<=4) {
    tslf++;
  }
  else
  {
    tslf=0;
    errorstatus &= ~(1 << 0); // Unset bit 0 (Clear GPS Error Condition Noted Switch to Max Performance Mode)
    errorstatus &= ~(1 << 1); // Unset bit 1 (Clear GPS Error Condition Noted Cold Boot GPS)
  }
wdt_reset(); // ...........................................................
  if((tslf>10 && ((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))) {
    setupGPS();
    wait(125);
    setGps_MaxPerformanceMode();
    wait(125);
    errorstatus |=(1 << 0); // Set Bit 1 (GPS Error Condition Noted Switch to Max Performance Mode)
    psm_status=0;
    errorstatus |=(1 << 4); // Set Bit 4 (Indicate PSM is disabled)
  }
  if(tslf>100 && ((errorstatus & (1 << 0))==1)&&((errorstatus & (1 << 1))==0)) {
    errorstatus |=(1 << 0); // Unset Bit 0 we've already tried that didn't work
    errorstatus |=(1 << 1); // Set bit 1 indicating we are cold booting the GPS
    Serial.flush();
    resetGPS();
    wait(125);
    setupGPS();
  }
wdt_reset(); // ...........................................................
}
// end main loop ##########################################################

// start initialise_interrupt function ++++++++++++++++++++++++++++++++++++
void initialise_interrupt()
{
wdt_reset(); // ...........................................................
  // initialize Timer5 so we cant use pin 46, 45, 44. timer is a 16-bit timer
  cli();          // disable global interrupts
  TCCR5A = 0;     // set entire TCCR5A register to 0
  TCCR5B = 0;     // same for TCCR5B
  OCR5A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR5B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR5B |= (1 << CS10);  // see in combination with below
  TCCR5B |= (1 << CS12);  // 1024 prescaler (C12 CS11 CS10 set to 1 0 1)
  // enable timer compare interrupt:
  TIMSK5 |= (1 << OCIE5A); // Interrupt mask register (TIMSKx)
  sei();          // enable global interrupts
}
// end initialise_interrupt function

// start ax25_init function +++++++++++++++++++++++++++++++++++++++++++++++
void ax25_init(void)
{
wdt_reset(); // ...........................................................
  /* Fast PWM mode, non-inverting output on OC2A */
  //TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  //TCCR2B = _BV(CS20);
  TCCR3B = TCCR3B & 0b11111000 | 0x01; // Sets fast PWM on pin 2 3 5 controled by timer3
  pinMode(HX1_TXD, OUTPUT);
}
// end ax25_init function

// start prepare data function ++++++++++++++++++++++++++++++++++++++++++++
void prepare_data() // (*@\label{code:fPrepareData}@*)
{
wdt_reset(); // ...........................................................
  if(aprstxstatus==0)
  {
    // get flight computer temp
    sensors.requestTemperatures();
    temperature1=sensors.getTempCByIndex(0);
    // Set or unset bit 2 indicating the temp sensor is faulty
    if(temperature1==-127) {errorstatus |=(1 << 2);}else{errorstatus &= ~(1 << 2);}
  }
  // gps set and get position
  gps_check_lock();
  gps_get_position();
  gps_get_time();
wdt_reset(); // ...........................................................
  // battery voltage
  batteryadc_v=analogRead(BATTERY_ADC)*4.8;
  battvsmooth[4] = battvsmooth[3];
  battvsmooth[3] = battvsmooth[2];
  battvsmooth[2] = battvsmooth[1];
  battvsmooth[1] = battvsmooth[0];
  battvsmooth[0] = batteryadc_v;
  battvaverage = (battvsmooth[0]+battvsmooth[1]+ battvsmooth[2]+battvsmooth[3]+battvsmooth[4])/5;
wdt_reset(); // ...........................................................
  // read light sensor if working, set bit 6 for error
  if(tsl.begin()) {
    lum1 = tsl.getFullLuminosity();
    ir1 = lum1 >> 16;
    full1 = lum1 & 0xFFFF; // visible is full - ir
    lux1 = tsl.calculateLux(full1, ir1);
    errorstatus &= ~(1 << 6);
    }
  else {
    lum1 = NAN;
    ir1 = NAN;
    full1 = NAN; // visible is full - ir
    lux1 = NAN;
    errorstatus |=(1 << 6);
  }
wdt_reset(); // ...........................................................
  // read temperature and humidity sensor if working, set bit 7 for error
  if(htu.begin()) {
    temperature3 = htu.readTemperature(); // in degree C
    humidity1 =htu.readHumidity(); // in RH %
    errorstatus &= ~(1 << 7);
    }
  else {
    temperature3 = NAN;
    humidity1 =NAN;
    errorstatus |=(1 << 7);
  }
wdt_reset(); // ...........................................................
  // read pressure and altimeter sensor if working, set bit 8 for error
  if(baro.begin()) {
    pressure1 = baro.getPressure(); // in Pa
    alt1 = baro.getAltitude(); // m
    temperature4 = baro.getTemperature(); // in degree C
    errorstatus &= ~(1 << 8);
    }
  else {
    pressure1 = NAN; // in Pa
    alt1 = NAN; // m
    temperature4 = NAN; // in degree C
    errorstatus |=(1 << 8);
  }
wdt_reset(); // ...........................................................
  // read precision temperature sensor if working, set bit 9 for error
  if(pretempsen.begin()) {
    temperature5 = pretempsen.readTempC(); // in degree C
    errorstatus &= ~(1 << 9);
    }
  else {
    temperature5 = NAN;
    errorstatus |=(1 << 9);
  }
wdt_reset(); // ...........................................................
  // read BMP085 pressure sensor if working, set bit 10 for error
  sensors_event_t bmp_event; // create sensor event
  bmp.getEvent(&bmp_event);
  if(bmp_event.pressure) {
    pressure2 = bmp_event.pressure; // in hPa
    pressure2 = pressure2*100; // in Pa
    alt2 = bmp.pressureToAltitude(1013.26,bmp_event.pressure); // m
    bmp.getTemperature(&temperature6); // in degree C
    errorstatus &= ~(1 << 10);
    }
  else {
    pressure2 = NAN; // in Pa
    alt2 = NAN; // m
    temperature6 = NAN; // in degree C
    errorstatus |=(1 << 10);
  }
wdt_reset(); // ...........................................................
  // read gyro IMU sensor if working, set bit 11 for error
  if(gyro.begin()) {
    // read gyro data in deg/s 5 times and make average
    gyro.read();gyroX=gyro.data.x-gyroXoff;gyroY=gyro.data.y-gyroYoff;gyroZ=gyro.data.z-gyroZoff; // 1
    gyro.read();gyroX+=gyro.data.x-gyroXoff;gyroY+=gyro.data.y-gyroYoff;gyroZ+=gyro.data.z-gyroZoff; // 2
    gyro.read();gyroX+=gyro.data.x-gyroXoff;gyroY+=gyro.data.y-gyroYoff;gyroZ+=gyro.data.z-gyroZoff; // 3
    gyro.read();gyroX+=gyro.data.x-gyroXoff;gyroY+=gyro.data.y-gyroYoff;gyroZ+=gyro.data.z-gyroZoff; // 4
    gyro.read();gyroX+=gyro.data.x-gyroXoff;gyroY+=gyro.data.y-gyroYoff;gyroZ+=gyro.data.z-gyroZoff; // 5
    gyroX=-gyroX/5;gyroY=-gyroY/5;gyroZ=gyroZ/5; // average
    errorstatus &= ~(1 << 11);
    } else {
    gyroX=NAN;
    gyroY=NAN;
    gyroZ=NAN;
    errorstatus |=(1 << 11);
  }
wdt_reset(); // ...........................................................
  // read accel IMU sensor if working, set bit 12 for error
  sensors_event_t acc_event; // create sensor event
  accel.getEvent(&acc_event);
  if(acc_event.acceleration.x) {
    // read accel data in m/s^2 5 times and make average
    accel.getEvent(&acc_event);accX=acc_event.acceleration.x;accY=acc_event.acceleration.y;accZ=acc_event.acceleration.z; // 1
    accel.getEvent(&acc_event);accX+=acc_event.acceleration.x;accY+=acc_event.acceleration.y;accZ+=acc_event.acceleration.z; // 2
    accel.getEvent(&acc_event);accX+=acc_event.acceleration.x;accY+=acc_event.acceleration.y;accZ+=acc_event.acceleration.z; // 3
    accel.getEvent(&acc_event);accX+=acc_event.acceleration.x;accY+=acc_event.acceleration.y;accZ+=acc_event.acceleration.z; // 4
    accel.getEvent(&acc_event);accX+=acc_event.acceleration.x;accY+=acc_event.acceleration.y;accZ+=acc_event.acceleration.z; // 5
    accX=-accX/5;accY=-accY/5;accZ=-accZ/5; // average
    errorstatus &= ~(1 << 12);
    } else {
    accX=NAN;
    accY=NAN;
    accZ=NAN;
    errorstatus |=(1 << 12);
  }
wdt_reset(); // ...........................................................
  // read mag IMU sensor if working, set bit 13 for error
  sensors_event_t mag_event; // create sensor event
  mag.getEvent(&mag_event);
  if(mag_event.magnetic.x) {
    // read mag data in micro Tesla 5 times and make average, we correct the values for offesets
    mag.getEvent(&mag_event);
    magX=magET[0][0]*(mag_event.magnetic.x-magEC[0])+magET[0][1]*(mag_event.magnetic.y-magEC[1])+magET[0][2]*(mag_event.magnetic.z-magEC[2]);
    magY=magET[1][0]*(mag_event.magnetic.x-magEC[0])+magET[1][1]*(mag_event.magnetic.y-magEC[1])+magET[1][2]*(mag_event.magnetic.z-magEC[2]);
    magZ=magET[2][0]*(mag_event.magnetic.x-magEC[0])+magET[2][1]*(mag_event.magnetic.y-magEC[1])+magET[2][2]*(mag_event.magnetic.z-magEC[2]); // 1
    mag.getEvent(&mag_event);
    magX+=magET[0][0]*(mag_event.magnetic.x-magEC[0])+magET[0][1]*(mag_event.magnetic.y-magEC[1])+magET[0][2]*(mag_event.magnetic.z-magEC[2]);
    magY+=magET[1][0]*(mag_event.magnetic.x-magEC[0])+magET[1][1]*(mag_event.magnetic.y-magEC[1])+magET[1][2]*(mag_event.magnetic.z-magEC[2]);
    magZ+=magET[2][0]*(mag_event.magnetic.x-magEC[0])+magET[2][1]*(mag_event.magnetic.y-magEC[1])+magET[2][2]*(mag_event.magnetic.z-magEC[2]); // 2
    mag.getEvent(&mag_event);
    magX+=magET[0][0]*(mag_event.magnetic.x-magEC[0])+magET[0][1]*(mag_event.magnetic.y-magEC[1])+magET[0][2]*(mag_event.magnetic.z-magEC[2]);
    magY+=magET[1][0]*(mag_event.magnetic.x-magEC[0])+magET[1][1]*(mag_event.magnetic.y-magEC[1])+magET[1][2]*(mag_event.magnetic.z-magEC[2]);
    magZ+=magET[2][0]*(mag_event.magnetic.x-magEC[0])+magET[2][1]*(mag_event.magnetic.y-magEC[1])+magET[2][2]*(mag_event.magnetic.z-magEC[2]); // 3
    mag.getEvent(&mag_event);
    magX+=magET[0][0]*(mag_event.magnetic.x-magEC[0])+magET[0][1]*(mag_event.magnetic.y-magEC[1])+magET[0][2]*(mag_event.magnetic.z-magEC[2]);
    magY+=magET[1][0]*(mag_event.magnetic.x-magEC[0])+magET[1][1]*(mag_event.magnetic.y-magEC[1])+magET[1][2]*(mag_event.magnetic.z-magEC[2]);
    magZ+=magET[2][0]*(mag_event.magnetic.x-magEC[0])+magET[2][1]*(mag_event.magnetic.y-magEC[1])+magET[2][2]*(mag_event.magnetic.z-magEC[2]); // 4
    mag.getEvent(&mag_event);
    magX+=magET[0][0]*(mag_event.magnetic.x-magEC[0])+magET[0][1]*(mag_event.magnetic.y-magEC[1])+magET[0][2]*(mag_event.magnetic.z-magEC[2]);
    magY+=magET[1][0]*(mag_event.magnetic.x-magEC[0])+magET[1][1]*(mag_event.magnetic.y-magEC[1])+magET[1][2]*(mag_event.magnetic.z-magEC[2]);
    magZ+=magET[2][0]*(mag_event.magnetic.x-magEC[0])+magET[2][1]*(mag_event.magnetic.y-magEC[1])+magET[2][2]*(mag_event.magnetic.z-magEC[2]); // 5
    magX=magX/5;magY=magY/5;magZ=magZ/5; // average
    errorstatus &= ~(1 << 13);
    } else {
    magX=NAN;
    magY=NAN;
    magZ=NAN;
    errorstatus |=(1 << 13);
  }
wdt_reset(); // ...........................................................
  // calculate roll, pitch and heading from acc and mag readings (*@\label{code:rphead}@*)
  // Normalize acceleration measurements so they range from 0 to 1
  accXnorm = -accX/sqrt(accX*accX+accY*accY+accZ*accZ);
  accYnorm = -accY/sqrt(accX*accX+accY*accY+accZ*accZ);
  // calculate pitch and roll in rad EQ. 10
  pitch = asin(-accXnorm);
  roll = asin(accYnorm/cos(pitch));
  // tilt compensated magnetic sensor measurements
  magXnorm = magX/sqrt(magX*magX+magY*magY+magZ*magZ);
  magYnorm = magY/sqrt(magX*magX+magY*magY+magZ*magZ);
  magZnorm = magZ/sqrt(magX*magX+magY*magY+magZ*magZ);
  // mag components EQ. 12
  magXcomp = magXnorm*cos(pitch)+magZnorm*sin(pitch);
  magYcomp = magXnorm*sin(roll)*sin(pitch)+magYnorm*cos(roll)-magZnorm*sin(roll)*cos(pitch);
  // calculate heading EQ. 13
  head = atan2(magYcomp,magXcomp)*180/PI;
  if (head < 0) {head +=360;}
  // transpose pitch and roll into degree
  pitch *= 180/PI;
  roll *= 180/PI;
wdt_reset(); // ...........................................................
}
// end prepare data function

// start writeToSD function +++++++++++++++++++++++++++++++++++++++++++++++
void writeToSD() // (*@\label{code:fwriteToSD}@*)
{
wdt_reset(); // ...........................................................
  dataFile = SD.open(logfilename, FILE_WRITE); // if file does not exist it will be created
  if (dataFile) {
      // write transmited string
      dataFile.print(callsign);dataFile.print(",");
      dataFile.print(count);dataFile.print(",");
      snprintf(tmpS,10,"%02d:%02d:%02d",hour,minute,second);
      dataFile.print(tmpS);dataFile.print(",");
      dataFile.print(lat < 0 ? "-" : "");
      dataFile.print(lat_int);dataFile.print(".");
      dataFile.print(lat_dec);dataFile.print(",");
      dataFile.print(lon < 0 ? "-" : "");
      dataFile.print(lon_int);dataFile.print(".");
      dataFile.print(lon_dec);dataFile.print(",");
      dataFile.print(maxalt);dataFile.print(",");
      dataFile.print(sats);dataFile.print(",");
      dataFile.print(temperature1);dataFile.print(","); // DS18B20
      dataFile.print(battvaverage);dataFile.print(",");
      dataFile.print(errorstatus);dataFile.print(",");
      // write remaining sensor data
      dataFile.print(lum1);dataFile.print(","); // TSL2561
      dataFile.print(ir1);dataFile.print(","); // TSL2561
      dataFile.print(full1);dataFile.print(","); // TSL2561
      dataFile.print(lux1);dataFile.print(","); // TSL2561
      dataFile.print(temperature3,2);dataFile.print(","); // HTU21DF
      dataFile.print(humidity1,2);dataFile.print(","); // HTU21DF
      dataFile.print(pressure1,0);dataFile.print(","); // MPL3115A2
      dataFile.print(alt1,2);dataFile.print(","); // MPL3115A2
      dataFile.print(temperature4,2);dataFile.print(","); // MPL3115A2
      dataFile.print(temperature5,2);dataFile.print(","); // MCP9808
      dataFile.print(pressure2,0);dataFile.print(","); // BMP085
      dataFile.print(alt2,2);dataFile.print(","); // BMP085
      dataFile.print(temperature6,2);dataFile.print(","); // BMP085
      dataFile.print(gyroX,0);dataFile.print(","); // L3GD20
      dataFile.print(gyroY,0);dataFile.print(","); // L3GD20
      dataFile.print(gyroZ,0);dataFile.print(","); // L3GD20
      dataFile.print(accX,2);dataFile.print(","); // LSM303 acc
      dataFile.print(accY,2);dataFile.print(","); // LSM303 acc
      dataFile.print(accZ,2);dataFile.print(","); // LSM303 acc
      dataFile.print(magX,2);dataFile.print(","); // LSM303 mag
      dataFile.print(magY,2);dataFile.print(","); // LSM303 mag
      dataFile.print(magZ,2);dataFile.print(","); // LSM303 mag
      dataFile.print(roll,2);dataFile.print(","); // LSM303 roll
      dataFile.print(pitch,2);dataFile.print(","); // LSM303 pitch
      dataFile.print(head,2);//dataFile.print(","); // LSM303 head
      dataFile.print("\n"); // EOL
      dataFile.close(); // close data file
      errorstatus &= ~(1 << 14);
  } else {errorstatus |=(1 << 14);}
wdt_reset(); // ...........................................................
}
// end writeToSD function

// start ISR timer5 function ++++++++++++++++++++++++++++++++++++++++++++++
ISR(TIMER5_COMPA_vect)
{
wdt_reset(); // ...........................................................
  if(alt>pedlim && sats >= 4)
  {
    digitalWrite(LED_WARN,LOW);
    digitalWrite(LED_OK,LOW);
  }
  else
  {
    currentMillis = millis();
    if(currentMillis - previousMillis > ONE_SECOND)
    {
      previousMillis = currentMillis;
      if(errorstatus!=8)
      {
        digitalWrite(LED_WARN,!digitalRead(LED_WARN));
        digitalWrite(LED_OK,LOW);
      }
      else
      {
        digitalWrite(LED_OK, !digitalRead(LED_OK));
        digitalWrite(LED_WARN,LOW);
      }
    }
  }
 // switch +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  switch(txstatus) {
wdt_reset(); // ...........................................................
  case 0: // This is the optional delay between transmissions.
wdt_reset(); // ...........................................................
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)) {
      txj=0;
      txstatus=1;
    }
    writeToSD(); // write to micro SD  (*@\label{code:writeToSD}@*)
    break;
  case 1: // Initialise transmission
wdt_reset(); // ...........................................................
    if(alt>maxalt && sats >= 4)
    {
      maxalt=alt;
    }
    lockvariables=1;
#ifndef APRS
    snprintf(txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%i,%i",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,temperature1,battvaverage,errorstatus);
#endif
#ifdef APRS
    snprintf(txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%i,%i,%i",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,temperature1,battvaverage,aprs_attempts,errorstatus);
#endif
    crccat(txstring);
    maxalt=0;
    lockvariables=0;
    txstringlength=strlen(txstring);
    txstatus=2;
    txj=0;
    break;
  case 2: // Grab a char and lets go transmit it.
wdt_reset(); // ...........................................................
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else
    {
      txstatus=0; // Should be finished
      txj=0;
      count++;
    }
    break;
  case 3:
wdt_reset(); // ...........................................................
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1);
      else rtty_txbit(0);
      txc = txc >> 1; // left shift one bit
      break;
    }
    else
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    }
  case 4:
wdt_reset(); // ...........................................................
    if(STOPBITS==2) // if two stop bits
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }
  }
}
// end ISR timer 1 function

// start ISR timer3 function ++++++++++++++++++++++++++++++++++++++++++++++
ISR(TIMER3_OVF_vect)
{
wdt_reset(); // ...........................................................
  static uint16_t phase  = 0;
  static uint16_t step   = PHASE_DELTA_1200;
  static uint16_t sample = 0;
  static uint8_t rest    = PREAMBLE_BYTES + REST_BYTES;
  static uint8_t byte;
  static uint8_t bit     = 7;
  static int8_t bc       = 0;
  /* Update the PWM output */
  OCR2B = pgm_read_byte(&_sine_table[(phase >> 7) & 0x1FF]);
  phase += step;

  if(++sample < SAMPLES_PER_BAUD) return;
  sample = 0;

  /* Zero-bit insertion */
  if(bc == 5)
  {
    step ^= PHASE_DELTA_XOR;
    bc = 0;
    return;
  }

  /* Load the next byte */
  if(++bit == 8)
  {
    bit = 0;

    if(rest > REST_BYTES || !_txlen)
    {
      if(!--rest)
      {
        /* Disable radio and interrupt */
        PORTD &= ~_BV(HX1_ENABLE); // Turn the HX1 Off
        aprstxstatus=0;
        TIMSK2 &= ~_BV(TOIE2);

        /* Prepare state for next run */
        phase = sample = 0;
        step  = PHASE_DELTA_1200;
        rest  = PREAMBLE_BYTES + REST_BYTES;
        bit   = 7;
        bc    = 0;
        return;
      }

      /* Rest period, transmit ax.25 header */
      byte = 0x7E;
      bc = -1;
    }
    else
    {
      /* Read the next byte from memory */
      byte = *(_txbuf++);
      if(!--_txlen) rest = REST_BYTES + 2;
      if(bc < 0) bc = 0;
    }
  }

  /* Find the next bit */
  if(byte & 1)
  {
    /* 1: Output frequency stays the same */
    if(bc >= 0) bc++;
  }
  else
  {
    /* 0: Toggle the output frequency */
    step ^= PHASE_DELTA_XOR;
    if(bc >= 0) bc = 0;
  }

  byte >>= 1;
}
// end ISR timer3 function

// some more functions ####################################################

// start rtty_txbit function ++++++++++++++++++++++++++++++++++++++++++++++
void rtty_txbit (int bit)
{
wdt_reset(); // ...........................................................
  if (bit) // bit is 1
  {
    analogWrite(MTX2_TXD, MTX2_OFFSET+((MTX2_SHIFT*1.8)/16)); // High
  }
  else // bit is 0
  {
    analogWrite(MTX2_TXD, MTX2_OFFSET); // Low
  }
}
// end rtty_txbit function

// start blinkled function ++++++++++++++++++++++++++++++++++++++++++++++++
void blinkled(int blinks)
{
wdt_reset(); // ...........................................................
  for(int blinkledx = 0; blinkledx <= blinks; blinkledx++) {
    digitalWrite(LED_WARN,HIGH);
    wait(100);
    digitalWrite(LED_WARN,LOW);
    wait(100);
  }
}
// end blinkled function

// start wait function, we redefine so make sure it works like we want ++++
void wait(unsigned long delaytime) // Arduino Delay doesn't get CPU Speeds below 8Mhz
{
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}
// end wait function

// start crccat function ++++++++++++++++++++++++++++++++++++++++++++++++++
uint16_t crccat(char *msg)
{
wdt_reset(); // ...........................................................
  uint16_t x;

  while(*msg == '$') msg++;

  for(x = 0xFFFF; *msg; msg++)
    x = _crc_xmodem_update(x, *msg);

  snprintf_P(msg, 8, PSTR("*%04X\n"), x);

  return(x);
}
// end crccat function

// EOF ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
