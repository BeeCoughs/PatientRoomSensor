// A simple timestamped data logger for sensors connected to Teensy4.1
// V1 Light, Sound, Movement
// V2 added temp sensor
// Brian Coughlin 2022

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <TimeLib.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include "Adafruit_MCP9808_Wire1.h"

#define LOG_INTERVAL  100 // mills between entries (reduce to take more/faster data)

/*  how many milliseconds before writing the logged data permanently to disk
  set it to the LOG_INTERVAL to write each time (safest)
  set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to
  the last 10 reads if power is lost but it uses less power and is much faster! */
#define SYNC_INTERVAL 10*LOG_INTERVAL
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port, good for testing/debugging sensors
#define WAIT_TO_START    0 // Wait for serial input in setup(), 0 for auto-run

#define redLEDpin 13    // the digital pin that connects to the LED on board

// PIR sensor
int PIRPin = 0;               // choose the digital input pin (for PIR sensor)
int pirState = LOW;           // Initialize pin state
int pirVal = 0;               // variable for reading the pin status

// Sound sensor
#define SoundSensorPin A0  //this pin read the analog voltage from the sound level meter
#define VREF  3.3  //voltage on AREF pin,default:operating voltage
double voltageValue, dbValue;

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// Light sensor
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

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
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);

  Serial.begin(9600);
  Serial.println();

  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);

  //
  pinMode(PIRPin, INPUT);     // declare sensor as input

  //  Serial.println(F(__DATE__));
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");

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

  logfile.println("datetime,tempC,dB,lux,PIR");

#if ECHO_TO_SERIAL
  Serial.println("datetime,tempC,dB,lux,PIR");
#endif //ECHO_TO_SERIAL

  // If you want to set the aref to something other than 5v
  //  analogReference(EXTERNAL);


  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");

    //Initialize temp sensor
  if (!tempsensor.begin(0x18)) {
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
    while (1);
  }

  tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
    
  }
}

void loop(void)
{
  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));
  
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }

  // Write the timestamp to the logfile
  logfile.print(hour());
  logDigits(minute());
  logDigits(second());
  logfile.print(" ");
  logfile.print(day());
  logfile.print(" ");
  logfile.print(month());
  logfile.print(" ");
  logfile.print(year()); 
#if ECHO_TO_SERIAL //print values over serial
digitalClockDisplay();  
#endif //ECHO_TO_SERIAL

tempsensor.wake();   // wake up, ready to read!
float c = tempsensor.readTempC();
  logfile.print(", ");
  logfile.print(c, 4);

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

  /* Get a PIR event */
  pirVal = digitalRead(PIRPin);  // read input value
  if (pirVal == HIGH) {            // check if the input is HIGH
    if (pirState == LOW) {
      // we have just turned on
      logfile.print(", ");
      logfile.print(pirVal);
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
    else {
      logfile.print(", ");
      logfile.print(pirVal);
    }
  } else {
    if (pirState == HIGH) {
      // we have just turned of
      logfile.print(", ");
      logfile.print(pirVal);
      // We only want to print on the output change, not state
      pirState = LOW;
    }
    else {
      logfile.print(", ");
      logfile.print(pirVal);
    }
  }

  logfile.println();
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(c, 4);
  Serial.print(", ");
  Serial.print(dbValue, 1);
  Serial.print(", ");
  Serial.print(event.light);
  Serial.print(", ");
  Serial.print(pirVal);
  Serial.println();
#endif // ECHO_TO_SERIAL


  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();

  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
}

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
void logDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  logfile.print(":");
  if(digits < 10)
    logfile.print('0');
  logfile.print(digits);
}
