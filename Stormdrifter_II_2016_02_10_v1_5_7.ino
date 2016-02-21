/*
   Program Code for Stormdrifter II by University of Osnabruck
   Based on:
   EURUS Balloon Code by James Coxon (GPS)
   Anthony Stirk's NTX2 RTTY code
   Tom Igoe's SD Library
   The Habduino code by Anthony Stirk.
   The DS18B20 library of Miles Burton.
   The SFE_BMP180 library of Sparkfun.com by Mike Grusin.
   The L3G and LSM303 libraries by Pololu.com.
   The HIH-6130 library by David Hagan.
   Version 1.5.7 of February 10th, 2016.
*/

/* The old program broke when trying to increment the counter at the beginning of the string any time the programmer was unplugged.
   Program probably was overloaded because it contained the code from Stormdrifter I that flew on March 5th, 2013 and the code for reading the Geiger Counter via Serial1.
   Let's start the program back from the roots and try to integrate it step-by-step (Geiger Counter omitted).
*/

/*
   Advanced to 1.5.7 to include launch detection and redo the fan shutdown and cutdown timer.
   Functional Description:
   Read the data from the GPS, check if the cutdown condition is met, read the temperature off the *two* DS18B20,
   the temperature and pressure from the BMP180, the temperature and humidity from the two HIH-6121,
   read the battery voltage, as well as the two analogue Humidity Sensors (HIH-5031),
   and the photodiode radiometer, display on UART1 and send via RTTY (adding CRC16 Checksum). Then save to SD.
*/

//***************LIBRARIES.************************//
#include <avr/io.h> //Used for AVR-C.
#include <avr/interrupt.h> //Used for Interrupt Control.
#include <stdlib.h> //Used for C operators.
#include <string.h> //For using the stringf command.
#include <util/crc16.h> //Library for CRC16 generation.
#include <SFE_BMP180.h> //Sparkfun BMP180 control library.
#include <Wire.h> //Two Wire Interface library (I2C).
#include <OneWire.h> //For OneWire Protocol.
#include <DallasTemperature.h> //Miles Burton's Temperature sensor library.
#include <SD.h> //SD Card library.
#include <Servo.h> //Servo library.
#include <L3G.h> // Gyroscope library.
#include <LSM303.h> // Accelerometer/Compass library.
#include <HIH6130.h> // Honeywell HIH-61xx library.
#include <SPI.h>  //Needed if code is to be compiled in Arduino 1.6.4
#include <EEPROM.h>

//******************Methods*************************//
void init_IO_pins(void);
void init_UART(void);
void init_BMP180(void);
void init_IMU(void);
void init_temperature_sensor(void);
void check_launch_rdy(void);
void getposition_gps(void);
void get_pressure(void);
void get_voltage(void);
void get_analogHumidity(void);
void get_HumidIcon(void);
void get_IMU(void);
void get_Radiometer(void);
void save_data(void);
void gather_send_data(void);
void check_termination_altitude(void);
void check_termination_time(void);
void check_FanShutdown(void);
void increase_counter(void);
uint16_t gps_CRC16_checksum (char *string);

//******************CONSTANTS.*******************//
//First the status LEDs.
#define LED1 1
#define LED2 22
//Define the Radio Control Pin and RTTY Data.
#define RADIOPIN 0
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 1       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 50    // Baud rate usually 50.
//Define the SPI Chip Select Pin for the SD Card.
#define chipSelect 4
//Define the control instance for the Servo Cutdown.
Servo CutServo;
//Define the hot-wire cutdown pin.
#define CUTPIN 3
//Define the Fan shutdown pin.
#define FANPIN 13
//Define the Launch readiness indicator pin.
#define RDYPin 14
//Navmode 6 -> up to 50 kilometer altitude.
#define NAVMODE_HIGH_ALTITUDE 6

//*****************VARIABLES.**********************//
int linennumber = 0;
//Define Buffers.
char Datastring[200]; //Full Datastring.
char Datastring2[200];
uint8_t buf[70]; // GPS receive buffer.
char checksum_str[6]; //CRC16 Checksum buffer.
char* Datastringpointer;
char* DatastringpointerLager;
char txstring[200];
char SDstring[200];
//GPS Variables.
int32_t lat = 0, lon = 0, alt = 0, lat_dec = 0, lon_dec = 0;
uint8_t hour = 0, minute = 0, second = 0, lock = 0, sats = 0;
unsigned long startGPS = 0;
int GPSerrorM = 0, GPSerrorL = 0, GPSerrorP = 0, GPSerrorT = 0, count = 0, n, gpsstatus, lockcount = 0;
byte navmode = 99;
byte error = 0;
int lon_int = 0, lat_int = 0;
//RTTY Variables
volatile int txstatus = 1;
volatile int txstringlength = 0;
volatile char txc;
volatile int txi;
volatile int txj;
//SD card file designator
File dataFile;
//Cutdown Indicator
boolean cutdown = false;
byte cut = 0; // Cutdown Message.
//Fan Shutdown Indicator
boolean fanSHDN = false;
byte fan = 1;
//Analog Variables
int intVolt = 0;
int BVolt = 0;
int32_t battvsmooth[5];
int anaHum0 = 0;
int anaHum1 = 0;
//IMU Variables
int GyroCalX = 0;
int GyroCalY = 0;
int GyroCalZ = 0;
int GyroX = 0;
int GyroY = 0;
int GyroZ = 0;

//************Temperature Sensor System.********************//
//The DS18B20 is on Pin 2, so let's tell the library about that.
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12
//Initiate OneWire Instance.
OneWire oneWire(ONE_WIRE_BUS);
//Pass Reference to the Dallas Temperature Library.
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses
DeviceAddress Thermometer0, Thermometer1;
//DS18B20 Variables
int TempInt0, TempInt1, CHECKSUM;
float celsius0 = 0, celsius1 = 0; //Temperature from the DS18B20.

//************Pressure Sensor System.********************//
//Create an instance of the BMP180 pressure sensor.
SFE_BMP180 pressure;
//BMP180 Variables
char status;
double T, P;
int IntTemp = 0;
long LongPress = 0;

//*********************IMU System.**********************//
L3G gyro;
LSM303 compass;

//*********************Humidity Sensor System.**********************//
HIH6130 rht0(0x27);
HIH6130 rht1(0x18);
int T_H0;
int T_H1;
int U_H0;
int U_H1;

//*********************Radiometer Humidity Sensor System.**********************//
int PV1;
int PV2;
int PV3;
int PV4;
int PV5;

//*********************CUTDOWN System (be careful!).**********************//
int32_t terminationAlt = 29000;
int32_t emergencyAlt = 29200;
int CloseToGround = 1000;
boolean launch_rdy = false;
int terminationCounter = 1950; //Calculated time from 0 km to 30 km on 5 m/s + 30 min reserve, expressed in line numbers which are about 4 sec apart.
int FanCounter = 3150; // 3.5 hours after counting starts, expressed in line numbers also.


//*********************SETUP ROUTINE.**********************//
void setup()
{
  init_IO_pins();
  init_UART();
  init_BMP180();
  init_IMU();
  setupGPS();
  delay(100);
  init_temperature_sensor;


  Datastringpointer = Datastring;
  DatastringpointerLager = Datastring;

  initialise_interrupt();
  if (!SD.begin(chipSelect)) return;

  Serial1.println("Startup Complete.");
}


//*********************MAIN LOOP.*************************//
void loop()
{
  getposition_gps();

  check_termination_altitude();

  check_termination_time();

  check_FanShutdown();

  get_pressure();

  get_temperature();

  get_HumidIcon();

  get_voltage();

  get_analogHumidity();

  get_IMU();

  get_Radiometer();

  gather_send_data();

  save_data();

  increase_counter();
}

/*
   I/O Pin Initialization.
   HARDWARE:
   VARIABLE:
*/
void init_IO_pins(void)
{
  pinMode(RADIOPIN, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(chipSelect, OUTPUT);
  pinMode(CUTPIN, OUTPUT);
  pinMode(FANPIN, OUTPUT);
  digitalWrite(FANPIN, HIGH);
  CutServo.attach(12);
  CutServo.writeMicroseconds(1950);
  pinMode(RDYPin, INPUT_PULLUP);
}


/*
   UART Initialization.
   HARDWARE:
   VARIABLE:
*/
void init_UART (void)
{
  Serial.begin(9600); //Establish connection to GPS.
  Serial.flush();
  Serial1.begin(9600); //Talk to the PC on UART1 @ 9600 bauds.
}

/*
   Interrupt Initialization.
   HARDWARE:
   VARIABLE:
*/
void initialise_interrupt()
{
  // initialize Timer2
  cli();          // disable global interrupts
  TCCR2A = 0;     // set entire TCCR1A register to 0
  TCCR2B = 0;     // same for TCCR1B
  OCR2A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR2A |= (1 << WGM21);   // turn on CTC mode:
  // Set CS20, CS21 and CS22 bits for:
  TCCR2B |= (1 << CS20);
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS22);
  // enable timer compare interrupt:
  TIMSK2 |= (1 << OCIE2A);
  sei();          // enable global interrupts
}

/*
   BMP180 Initialization.
   HARDWARE: BMP180
   VARIABLE:
*/
void init_BMP180(void)
{
  if (pressure.begin())
    Serial1.println("BMP180 init success");
  else
  {
    Serial1.println("BMP180 init fail\n\n");
    //   while(1); // Pause forever.
  }
}

/*
   Start DS18B20 communications. It's the Temperature sensor.
   HARDWARE: DS18B20
   VARIABLE:
*/
void init_temperature_sensor(void)
{
  sensors.begin();
  if (!sensors.getAddress(Thermometer0, 0)) return;
  if (!sensors.getAddress(Thermometer1, 1)) return;
  sensors.setResolution(Thermometer0, TEMPERATURE_PRECISION);
  sensors.setResolution(Thermometer1, TEMPERATURE_PRECISION);
}

/*
   Start Pololu MinIMU v3 Gyroscope and Accelerometer/Compass
   HARDWARE: L3G & LMS303
   VARIABLE:
*/
void init_IMU(void)
{
  compass.init();
  compass.enableDefault();
  gyro.init();
  gyro.enableDefault();

  GyroCalX = word(EEPROM.read(1), EEPROM.read(0));
  GyroCalY = word(EEPROM.read(3), EEPROM.read(2));
  GyroCalZ = word(EEPROM.read(5), EEPROM.read(4));
}

/*
   Setting up the GPS and collecting infos from it
   HARDWARE: GPS (uBlox MAX-7Q)
   VARIABLE: navmode, setNav[], error
*/
void getposition_gps (void)
{
  gps_check_nav();
  if (navmode != NAVMODE_HIGH_ALTITUDE) {
    Serial.flush();
    // Check and set the navigation mode (Airborne, 1G).
    uint8_t setNav[] = {
      0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
    };
    sendUBX(setNav, sizeof(setNav) / sizeof(uint8_t));
  }
  gps_check_lock(); // Check if the GPS has Lock.
  gps_get_position(); // Read the Latitude, Longitude and Altitude.
  gps_get_time(); // Read the current Time.
  error = GPSerrorM + GPSerrorL + GPSerrorP + GPSerrorT; // Add up the errors.
  print_latitude(); // Convert the Latitude into the proper format.
  print_longitude(); // Convert the Longitude into the proper format.
  if (lock == 3) digitalWrite(LED2, LOW);
  else digitalWrite(LED2, HIGH);
}

/*
   NTX2B Interrupt Routine.
   HARDWARE: NTX2B
   VARIABLE:
*/
ISR(TIMER2_COMPA_vect)
{
  switch (txstatus) {
    case 0: // This is the optional delay between transmissions.
      txj++;
      if (txj > (TXDELAY * RTTY_BAUD)) {
        txj = 0;
        txstatus = 1;
      }
      break;
    case 1: // Initialise transmission, take a copy of the string so it doesn't change mid transmission.
      strcpy(txstring, DatastringpointerLager);
      txstringlength = strlen(txstring);
      txstatus = 2;
      txj = 0;
      break;
    case 2: // Grab a char and lets go transmit it.
      if ( txj < txstringlength)
      {
        txc = txstring[txj];
        txj++;
        txstatus = 3;
        rtty_txbit (0); // Start Bit;
        txi = 0;
      }
      else
      {
        txstatus = 0; // Should be finished
        txj = 0;
      }
      break;
    case 3:
      if (txi < ASCII)
      {
        txi++;
        if (txc & 1) rtty_txbit(1);
        else rtty_txbit(0);
        txc = txc >> 1;
        break;
      }
      else
      {
        rtty_txbit (1); // Stop Bit
        txstatus = 4;
        txi = 0;
        break;
      }
    case 4:
      if (STOPBITS == 2)
      {
        rtty_txbit (1); // Stop Bit
        txstatus = 2;
        break;
      }
      else
      {
        txstatus = 2;
        break;
      }

  }
}

/*
   NTX2B Driving Routine.
   HARDWARE: NTX2B
   VARIABLE:
*/
void rtty_txbit (int bit)
{
  if (bit)
  {
    digitalWrite(RADIOPIN, HIGH); // High
    digitalWrite(LED1, HIGH);
  }
  else
  {
    digitalWrite(RADIOPIN, LOW); // Low
    digitalWrite(LED1, LOW);
  }
}

/*
   Get Pressure
   HARDWARE: BMP180
   VARIABLE: T,P,p0,status,IntTemp
*/
void get_pressure(void)
{
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      IntTemp = T * 100;

      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          LongPress = P * 100;
        }

      }

    }

  }
  Serial1.println(LongPress);
}

/*
   Get Temperature
   HARDWARE: DS18B20
   VARIABLE: celsius0, celsius1, TempInt0, TempInt1
*/
void get_temperature(void)
{
  sensors.requestTemperatures(); //Call the function to request T from DS18B20.
  celsius0 = sensors.getTempCByIndex(0);
  TempInt0 = celsius0 * 100; //Transform the float variable into an integer.
  celsius1 = sensors.getTempCByIndex(1);
  TempInt1 = celsius1 * 100; //Transform the float variable into an integer.
  Serial1.println(celsius0);
  Serial1.println(celsius1);
}

/*
   Get Temperature and Humidity
   HARDWARE: HIH-6121
   VARIABLE: T_H0, U_H0, T_H1, U_H1
*/
void get_HumidIcon(void)
{
  rht0.readRHT();
  T_H0 = rht0.temperature * 100;
  U_H0 = rht0.humidity * 10;
  rht1.readRHT();
  T_H1 = rht1.temperature * 100;
  U_H1 = rht1.humidity * 10;
}

/*
   Get Battery Voltage
   HARDWARE: Voltage Divider
   VARIABLE: intVolt, BVolt
*/
void get_voltage(void)
{
  intVolt = analogRead(6);
  battvsmooth[4] = battvsmooth[3];
  battvsmooth[3] = battvsmooth[2];
  battvsmooth[2] = battvsmooth[1];
  battvsmooth[1] = battvsmooth[0];
  battvsmooth[0] = intVolt;
  BVolt = (battvsmooth[0] + battvsmooth[1] + battvsmooth[2] + battvsmooth[3] + battvsmooth[4]) / 5;
}

/*
   Get Humidity from the two HIH-5031
   HARDWARE: HIH-5031_0, HIH-5031_1
   VARIABLE: intHum0, intHum1, anaHum0, anaHum1
*/
void get_analogHumidity(void)
{
  anaHum0 = analogRead(7);
  anaHum1 = analogRead(5);
}

/*
   Get Motion Data
   HARDWARE: L3G & LSM303
   VARIABLE: compass.a.x, compass.a.y, compass.a.z, gyro.g.x, gyro.g.y, gyro.g.z, compass.m.x, compass.m.y, compass.m.z
*/
void get_IMU(void)
{
  gyro.read();
  compass.read();

  GyroX = gyro.g.x - GyroCalX;
  GyroY = gyro.g.y - GyroCalY;
  GyroZ = gyro.g.z - GyroCalZ;
}

/*
   Get Radiometer Data
   HARDWARE: VTB8440BH Photodiode & Atmel AtTiny841
   VARIABLE: PV1, PV2, PV3, PV4, PV5
*/

void get_Radiometer(void)
{
  int n = Wire.requestFrom(2, 10);    // request 10 bytes from slave device #2
  for (int i = 0; i < n; i++)
  {
    buf[i] = Wire.read();
  }

  PV1 = word(buf[1], buf[0]);
  PV2 = word(buf[3], buf[2]);
  PV3 = word(buf[5], buf[4]);
  PV4 = word(buf[7], buf[6]);
  PV5 = word(buf[9], buf[8]);
}

/*
   Link all data to string and send it over UART1 and Radio
   HARDWARE: GPS, RADIO
   VARIABLE: linennumber, hour, minute, second, lat, lat_int, lat_dec, lon,
             lon_int,lon_dec, alt, navmode, error, lock, sats, IntTemp, LongPress, TempInt0, TempInt1, anaHum0, anaHum1, BVolt, compass.a.x, compass.a.y, compass.a.z,
             gyro.g.x, gyro.g.y, gyro.g.z, compass.m.x, compass.m.y, compass.m.z
*/
void gather_send_data(void)
{
  sprintf(Datastringpointer, "$$$$$OERNEN-II,%i,%02d:%02d:%02d,%s%i.%07ld,%s%i.%07ld,%ld,%d,%d,%d,%d,%d,%lu,%d,%d,%u,%u,%u,%6d,%d,%d",
          linennumber, hour, minute, second, lat < 0 ? "-" : "", lat_int, lat_dec, lon < 0 ? "-" : "",
          lon_int, lon_dec, alt, navmode, error, lock, sats, IntTemp, LongPress, TempInt0, TempInt1, anaHum0, anaHum1, BVolt, compass.a.z, cut, fan); //Build the Datastring.
  CHECKSUM = gps_CRC16_checksum(Datastringpointer);  // Calculates the checksum for this datastring

  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(Datastringpointer, checksum_str);

  Serial1.println(Datastringpointer); //Send the Datastring on UART1.

  sprintf(SDstring, "$$$$$OERNEN-II,%i,%02d:%02d:%02d,%s%i.%07ld,%s%i.%07ld,%ld,%d,%d,%d,%d,%d,%lu,%d,%d,%u,%u,%u,%d,%d,%u,%u,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%u,%u,%u,%u,%u,%d,%d",
          linennumber, hour, minute, second, lat < 0 ? "-" : "", lat_int, lat_dec, lon < 0 ? "-" : "",
          lon_int, lon_dec, alt, navmode, error, lock, sats, IntTemp, LongPress, TempInt0, TempInt1, anaHum0, anaHum1, BVolt, T_H0, T_H1, U_H0, U_H1, compass.a.x, compass.a.y, compass.a.z,
          GyroX, GyroY, GyroZ, compass.m.x, compass.m.y, compass.m.z, PV1, PV2, PV3, PV4, PV5, cut, fan); //Build the SDstring for saving.

  Serial1.println(SDstring); //Send the Datastring on UART1.

  //Vertausche strings dmit einer bschrieben und einer geschickt wird
  if (Datastringpointer == Datastring) {
    Datastringpointer = Datastring2;
    DatastringpointerLager = Datastring;
  } else {
    Datastringpointer = Datastring;
    DatastringpointerLager = Datastring2;
  }
}

/*
   Save data to SD-card
   HARDWARE: SD-card
   VARIABLE: dataFile, SDstring
*/
void save_data(void)
{
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(SDstring);
    dataFile.close();
  }
}

/*
   Check if Cutdown/Termination Condition is met.
   HARDWARE: Servo Controller
   VARIABLE: cutdown, alt
*/
void check_termination_altitude(void)
{
  if (lock != 3 && sats == 0 || alt <= CloseToGround)
  {
    CutServo.writeMicroseconds(1950);
    digitalWrite(CUTPIN, LOW);
    cut = 1;
  }
  else if (lock == 3 && sats != 0 && alt >= terminationAlt) // Is the Altitude greater than X m? Then Activate the Cutdown.
  {
    CutServo.writeMicroseconds(1000);
    cutdown = true;
    cut = 2;
  }
  if (lock == 3 && sats != 0 && alt >= emergencyAlt) // Is the Altitude greater than X+1000 m? Then Activate the hot-wire Cutdown.
  {
    digitalWrite(CUTPIN, HIGH);
    cut = 3;
  }
  else if (alt < terminationAlt && 0>= cut <= 3)
  {
    CutServo.writeMicroseconds(1950);
    digitalWrite(CUTPIN, LOW);
  }
}

void check_termination_time(void)
{
  if (linennumber > terminationCounter)
  {
    digitalWrite(CUTPIN, HIGH);
    cut = 4;
  }
  else
  {
    digitalWrite(CUTPIN, LOW);
  }
}

/*
   Check if Fan shutdown condition is met.
   HARDWARE: Pentium III Fan
   VARIABLE: fanSHDN, alt
*/
void check_FanShutdown(void)
{
  if (linennumber > FanCounter)
  {
    digitalWrite(FANPIN, LOW);
    fan = 0;
  }
  else
  {
    digitalWrite(FANPIN, HIGH);
  }
}

/*
   Checksum Generator
   generates CRC16 Checksum
   HARDWARE:
   VARIABLE:
*/
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first five $s
  for (i = 5; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

void increase_counter(void)
{
  if (digitalRead(RDYPin) == 1)
  {
    launch_rdy = true;
  }
  
  if (launch_rdy == true)
  {
    linennumber++; //Increase the Line Count by 1.
  }
  else
  {
    linennumber = 0;
  }
}
