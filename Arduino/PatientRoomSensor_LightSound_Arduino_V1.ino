// A data logger for the Arduino-based patient room sensor boxes
// Brian Coughlin 2022

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  100 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 10*LOG_INTERVAL // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

// Sound sensor
#define SoundSensorPin A1  //this pin read the analog voltage from the sound level meter
#define VREF  5.0  //voltage on AREF pin,default:operating voltage

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

double voltageValue, dbValue;

RTC_PCF8523 rtc; // define the Real Time Clock object
// #define PCF8523_ADDRESS 0x68  // I2C address for clock pin for making calibration adjustment
// #define mode 1  // Adjust every minute
//#define OFFSET 24  //Number of adjustment pulses, calculated from RTC datasheet
// enum Pcf8523OffsetMode { PCF8523_TwoHours = 0x00, PCF8523_OneMinute = 0x80 };

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

void error(const char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while (1);
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println("init");

  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);

//  Serial.println(F(__DATE__));
  tsl.setGain(TSL2561_GAIN_16X);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);

#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

//  // initialize the SD card
//  Serial.print("Initializing SD card...");
//  // make sure that the default chip select pin is set to
//  // output, even if you don't use it:
//  pinMode(10, OUTPUT);
//
//  // see if the card is present and can be initialized:
//  if (!SD.begin(chipSelect)) {
//    error("Card failed, or not present");
//  }
//  Serial.println("card initialized.");

  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }

  if (! logfile) {
    error("couldnt create file");
  }

  Serial.print("Logging to: ");
  Serial.println(filename);

//Calibrate real-time clock with offset pulses
//  uint8_t reg = (uint8_t) offset & 0x7F;
//  reg |= mode;
//
//  Wire.beginTransmission(PCF8523_ADDRESS);
//  Wire.write(offset);
//  Wire.write(reg);
//  Wire.endTransmission();

  // connect to RTC
  Wire.begin();
  if (!rtc.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }

//    Serial.print("Sending RTC calibration pulse...");
//rtc.calibrate(PCF8523_TwoHours, OFFSET);
//  Serial.print("Sent!");

  logfile.println("datetime,dB, lux");

#if ECHO_TO_SERIAL
  Serial.println("datetime,dB, lux");
#endif //ECHO_TO_SERIAL

  // If you want to set the aref to something other than 5v
  //  analogReference(EXTERNAL);


//The following lines set the clock. Upload once with the following line set as "if (1)"
//Then change to "if (0)" and upload again. Do not open the serial monitor between these actions
//  if (! rtc.initialized()) {
if (0) {
    Serial.println("RTC update enabled");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  DateTime t = DateTime(rtc.now().unixtime() + 11);  // Change adjustment based on second offset
  rtc.adjust(t);
  }

}

void loop(void)
{
  // DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));

  digitalWrite(greenLEDpin, HIGH);
  // fetch the time
  DateTime now = rtc.now();
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  // logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
#endif //ECHO_TO_SERIAL

  // obtain sound information
  voltageValue = analogRead(SoundSensorPin) / 1024.0 * VREF;
  dbValue = voltageValue * 50.0;  //convert voltage to decibel value
  logfile.print(", ");
  logfile.print(dbValue);

  /* Get a new light sensor event */
  sensors_event_t event;
  tsl.getEvent(&event);

  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    //   Serial.print(event.light); Serial.print(" lux");
    logfile.print(", ");
    logfile.print(event.light);
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }


#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(dbValue, 1);
  Serial.print(", ");
  Serial.print(event.light);
#endif //ECHO_TO_SERIAL

  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();

  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);

}
