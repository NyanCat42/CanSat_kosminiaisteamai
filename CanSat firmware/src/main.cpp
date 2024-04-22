#include <Arduino.h>
#include <PMS7003.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "Adafruit_AGS02MA.h"
#include <MPU6500_WE.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <DFRobot_QMC5883.h>
#include <SoftwareSerial.h>
#include "LoRa_E220.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"


//Pin Defines:
#define PMSRX 39
#define PMSTX 33
#define SERVO1PIN 12
#define SERVO2PIN 14
#define SERVO3PIN 27
#define DHTPIN 4
#define GPSRX 36
#define GPSTX 32
#define BEEPER 2
#define m0 15
#define m1 25
#define aux 26
#define O2RX 34
#define CO2RX 35

// // Softserial:

 EspSoftwareSerial::UART pmsSerial;
 EspSoftwareSerial::UART co2Serial;
 EspSoftwareSerial::UART o2Serial;

/// Particle sensor defines:

GuL::PMS7003 pms(pmsSerial);


// Servo defines:                           

Servo servo1;
Servo servo2;
Servo servo3;
int servoPosition1;
int servoPosition2;
int servoPosition3;

//Humidity sensor defines:

#define DHTTYPE    DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);
int temperatureReading = 0;
int humidityReading = 0;

// GPS defines:

SFE_UBLOX_GNSS myGNSS;
long lastTime = 0;

// tvoc sensor defines:

Adafruit_AGS02MA ags;
uint32_t tvoc = 0;

// Gyroscope; accelerometer defines

#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

// Barometer defines

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// compas defines

DFRobot_QMC5883 compass(&Wire, 0x0D);

// Timers:

long lastTime1 = 0;
long lastTime2 = 0;
long lastTime3 = 0;
long lastTime4 = 0;
long lastTime5 = 0;

// Radio defines

LoRa_E220 e220ttl(16, 17, &Serial2, -1, 15, 15, UART_BPS_RATE_9600, SERIAL_8N1);
bool setRadioParameters = 0;
byte radioAddressLo = 0x03;
byte radioAddressHi = 0x00;
int RadioChannel = 23;
bool found42069 = true;


// CO2 sensor defines:

const int bufferSizeCO2 = 7;
byte bufferCO2[bufferSizeCO2];
int bufferIndexCO2 = 0;
bool found0x42 = false;
int co2concentration = 0;

// O2 sensor defines:

const int bufferSizeO2 = 7;
byte bufferO2[bufferSizeO2];
int bufferIndexO2 = 0;
bool found0xFF = false;
int o2concentration = 0;


void setup() {

  Serial.begin(115200);

  // Radio setup

	e220ttl.begin();

  // Particle sensor setup:

  pmsSerial.begin(9600, SWSERIAL_8N1, PMSRX, PMSTX, false);
  if (!pmsSerial) { // If the object did not initialize, then its configuration is invalid
  Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
  while (1) { // Don't continue with invalid configuration
    delay (1000);
    }
  } 

  pms.setToPassiveReporting();

  // Servo setup:

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo1.attach(SERVO1PIN, 900, 2100);
  servo2.attach(SERVO2PIN, 900, 2100);
  servo3.attach(SERVO3PIN, 900, 2100);

  // Humidity sensor setup

  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  Serial.print("min_delay: ");
  Serial.println(sensor.min_delay);

  // GPS setup:


  do {
    Serial.println("GNSS: trying 38400 baud");
    Serial1.begin(38400, SERIAL_8N1, GPSRX, GPSTX);
    if (myGNSS.begin(Serial1) == true) break;

    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    Serial1.begin(9600, SERIAL_8N1, GPSRX, GPSTX);
    if (myGNSS.begin(Serial1) == true) {
        Serial.println("GNSS: connected at 9600 baud, switching to 38400");
        myGNSS.setSerialRate(38400);
        delay(100);
    } else {
        //myGNSS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.println("GNSS serial connected");

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR


  // TVOC sensor setup:

  if (! ags.begin(&Wire, 0x1A)) {
    Serial.println("Couldn't find AGS20MA sensor, check your wiring and pullup resistors!");
    while (1) yield();
  }

  if (ags.getFirmwareVersion() == 0) {
    Serial.println(F("Could not read firmware, I2C communications issue?"));
    while (1) yield();
  }

  Serial.print("Firmware version: 0x");
  Serial.println(ags.getFirmwareVersion(), HEX);
  ags.printSensorDetails();

  // Beeper setup:

  pinMode(BEEPER, OUTPUT);
  digitalWrite(BEEPER, LOW);

  //gyro setup

  Wire.begin();
  if(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
  }
  else{
    Serial.println("MPU6500 is connected");
  }

  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");

  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);


  // barometer seutp

  unsigned status;
  status = bmp.begin(0x76);

    /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

  //compass setup

  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }

  if(compass.isHMC())
  {
    Serial.println("Initialize HMC5883");

    //Set/get the compass signal gain range, default to be 1.3 Ga
    // compass.setRange(HMC5883L_RANGE_1_3GA);
     Serial.print("compass range is:");
     Serial.println(compass.getRange());

    //Set/get measurement mode
    // compass.setMeasurementMode(HMC5883L_CONTINOUS);
     Serial.print("compass measurement mode is:");
     Serial.println(compass.getMeasurementMode());

    //Set/get the data collection frequency of the sensor
    // compass.setDataRate(HMC5883L_DATARATE_15HZ);
     Serial.print("compass data rate is:");
     Serial.println(compass.getDataRate());

    //Get/get sensor status
    // compass.setSamples(HMC5883L_SAMPLES_8);
     Serial.print("compass samples is:");
     Serial.println(compass.getSamples());
  }
  else if(compass.isQMC())
  {
    Serial.println("Initialize QMC5883");
     compass.setRange(QMC5883_RANGE_2GA);
     Serial.print("compass range is:");
     Serial.println(compass.getRange());

     compass.setMeasurementMode(QMC5883_CONTINOUS);
     Serial.print("compass measurement mode is:");
     Serial.println(compass.getMeasurementMode());

     compass.setDataRate(QMC5883_DATARATE_50HZ);
     Serial.print("compass data rate is:");
     Serial.println(compass.getDataRate());

     compass.setSamples(QMC5883_SAMPLES_8);
     Serial.print("compass samples is:");
     Serial.println(compass.getSamples());
  }
  else if(compass.isVCM())
  {
    Serial.println("Initialize VCM5883L");
    // compass.setMeasurementMode(VCM5883L_CONTINOUS);
     Serial.print("compass measurement mode is:");
     Serial.println(compass.getMeasurementMode());

    // compass.setDataRate(VCM5883L_DATARATE_200HZ);
     Serial.print("compass data rate is:");
     Serial.println(compass.getDataRate());
  }



// SD card setup:

SPI.begin(18, 23, 19, 5);
SPI.setDataMode(SPI_MODE0);

    if(!SD.begin(5)){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

// O2 sensor setup:

  o2Serial.begin(9600, SWSERIAL_8N1, O2RX, -1, false);
  o2Serial.enableIntTx(false);

// CO2 sensor setup:

  co2Serial.begin(9600, SWSERIAL_8N1, CO2RX, -1, false);
  co2Serial.enableIntTx(false);

}

void loop() {

    
  // Read CO2 sensor
  if (co2Serial.available()) {
    byte incomingByteCO2 = co2Serial.read();

    if (incomingByteCO2 == 0x42) {
      found0x42 = true;
    } else if (found0x42) {
      bufferCO2[bufferIndexCO2++] = incomingByteCO2;
      if (bufferIndexCO2 == 7) {
        co2concentration = (bufferCO2[5] << 8) + bufferCO2[6]; // i showed dad code, he said this would be a more "idealistic" way of doing things idk what << does
        bufferIndexCO2 = 0;
        found0x42 = false;
      }
    }
  }

  // Read O2 sensor
  if (o2Serial.available()) {
    byte incomingByteO2 = o2Serial.read();

    if (incomingByteO2 == 0xFF) {
      found0xFF = true;
    } else if (found0xFF) {
      bufferO2[bufferIndexO2++] = incomingByteO2;
      if (bufferIndexO2 == 3) {
        byte hiByteO2 = bufferO2[1];
        byte loByteO2 = bufferO2[2];
        o2concentration = hiByteO2 * 256 + loByteO2;
        bufferIndexO2 = 0;
        found0xFF = false;
      }
    }
  }

  // Listen for radio commands
  if (Serial2.available()) {
    int incomingCommand = Serial2.read();
    if (incomingCommand == 42069) {
      found42069 = true;
      Serial2.println("enter servo position (0-180)");
    } else if (found42069) {
      servoPosition1 = incomingCommand;
      found0xFF = false;
    }
  }


  // Read sensors
  if (millis() - lastTime > 2000)
  {
    lastTime = millis(); //Update the timer

    // UV sensoe read:
    int uvIndex = analogRead(13);

    // Particle sensor read:
    pms.poll();
    pms.read();
    
    // GPS read:
    long latitude = myGNSS.getLatitude();
    long longitude = myGNSS.getLongitude();
    long altitude = myGNSS.getAltitude();
    byte SIV = myGNSS.getSIV();

    // Gyroscope, accelerometer read:
    xyzFloat gValue = myMPU6500.getGValues();
    xyzFloat gyr = myMPU6500.getGyrValues();
    float temp = myMPU6500.getTemperature();
    float resultantG = myMPU6500.getResultantG(gValue);

    // Barometer read:
    sensors_event_t temp_event, pressure_event;
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);


    // Magnetometer read:
    float declinationAngle = (8.0 + (35.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);
    sVector_t mag = compass.readRaw();
    compass.getHeadingDegrees();

    // Read slow sensor
    if (millis() - lastTime1 > 2000)
    {

      // Humidity sensor read:
      sensors_event_t event;
      dht.temperature().getEvent(&event);
      dht.humidity().getEvent(&event);
      temperatureReading = event.temperature;
      humidityReading = event.relative_humidity;

      lastTime1 = millis(); //Update the timer
      
    }

    // Read veery slow sensor
    if (millis() - lastTime2 > 10000)
    {
      lastTime2 = millis(); //Update the timer

      // TVOC sensor read:
      tvoc = ags.getTVOC();

    }

    if (millis() - lastTime3 > 10000) {
      lastTime3 = millis(); //Update the timer
      Serial.print("UV index: ");
      Serial.println(uvIndex);
      Serial.print("PM1 standarized (µg/m3): ");
      Serial.println(pms.getPM1_STD());
      Serial.print("PM1 standarized (µg/m3): ");
      Serial.println(pms.getPM1_STD());
      Serial.print("PM2.5 standarized (µg/m3): ");
      Serial.println(pms.getPM2_5_STD());
      Serial.print("PM10 standarized (µg/m3): ");
      Serial.println(pms.getPM10_STD());
      Serial.print("PM1 atmospheric (µg/m3): ");
      Serial.println(pms.getPM1_ATM());
      Serial.print("PM2.5 atmospheric (µg/m3): ");
      Serial.println(pms.getPM2_5_ATM());
      Serial.print("PM10 atmospheric (µg/m3): ");
      Serial.println(pms.getPM10_ATM());
      Serial.print("PN300 (#/0.1l): ");
      Serial.println(pms.getCntBeyond300nm());
      Serial.print("PN500 (#/0.1l): ");
      Serial.println(pms.getCntBeyond500nm());
      Serial.print("PN1000 (#/0.1l): ");
      Serial.println(pms.getCntBeyond1000nm());
      Serial.print("PN2500 (#/0.1l): ");
      Serial.println(pms.getCntBeyond2500nm());
      Serial.print("PN5000 (#/0.1l): ");
      Serial.println(pms.getCntBeyond5000nm());
      Serial.print("PN10000 (#/0.1l): ");
      Serial.println(pms.getCntBeyond10000nm());
      Serial.print("Temperature DHT11 (C): ");
      Serial.println(temperatureReading);
      Serial.print("Humidity (%): ");
      Serial.println(humidityReading);
      Serial.print("Latitude (degrees*10^-7): ");
      Serial.println(latitude);
      Serial.print("Longitude (degrees*10^-7): ");
      Serial.println(longitude);
      Serial.print("Altitude (mm): ");
      Serial.println(altitude);
      Serial.print("Satelites in view: ");
      Serial.println(SIV);
      Serial.print("TVOC (ppb): ");
      Serial.println(tvoc);
      Serial.print("Acceleration x,y,z (g): ");
      Serial.print(gValue.x);
      Serial.print(", ");
      Serial.print(gValue.y);
      Serial.print(", ");
      Serial.println(gValue.z);
      Serial.print("Resultant g: ");
      Serial.println(resultantG);
      Serial.println("Gyroscope x,y,z (degrees/s): ");
      Serial.print(gyr.x);
      Serial.print(", ");
      Serial.print(gyr.y);
      Serial.print(", ");
      Serial.println(gyr.z);
      Serial.print("Temperature MPU6500 (C): ");
      Serial.println(temp);
      Serial.print("Temperature BMP280 (C): ");
      Serial.println(temp_event.temperature);
      Serial.print("Pressure (hpa): ");
      Serial.println(pressure_event.pressure);
      Serial.print("Heading (degrees): ");
      Serial.println(mag.HeadingDegress);
      Serial.print("CO2 (ppb): ");
      Serial.println(co2concentration);
      Serial.print("O2 (%): ");
      Serial.println(o2concentration);

      Serial1.print("UV index: ");
      Serial1.println(uvIndex);
      Serial1.print("PM1 standarized (µg/m3): ");
      Serial1.println(pms.getPM1_STD());
      Serial1.print("PM1 standarized (µg/m3): ");
      Serial1.println(pms.getPM1_STD());
      Serial1.print("PM2.5 standarized (µg/m3): ");
      Serial1.println(pms.getPM2_5_STD());
      Serial1.print("PM10 standarized (µg/m3): ");
      Serial1.println(pms.getPM10_STD());
      Serial1.print("PM1 atmospheric (µg/m3): ");
      Serial1.println(pms.getPM1_ATM());
      Serial1.print("PM2.5 atmospheric (µg/m3): ");
      Serial1.println(pms.getPM2_5_ATM());
      Serial1.print("PM10 atmospheric (µg/m3): ");
      Serial1.println(pms.getPM10_ATM());
      Serial1.print("PN300 (#/0.1l): ");
      Serial1.println(pms.getCntBeyond300nm());
      Serial1.print("PN500 (#/0.1l): ");
      Serial1.println(pms.getCntBeyond500nm());
      Serial1.print("PN1000 (#/0.1l): ");
      Serial1.println(pms.getCntBeyond1000nm());
      Serial1.print("PN2500 (#/0.1l): ");
      Serial1.println(pms.getCntBeyond2500nm());
      Serial1.print("PN5000 (#/0.1l): ");
      Serial1.println(pms.getCntBeyond5000nm());
      Serial1.print("PN10000 (#/0.1l): ");
      Serial1.println(pms.getCntBeyond10000nm());
      Serial1.print("Temperature DHT11 (C): ");
      Serial1.println(temperatureReading);
      Serial1.print("Humidity (%): ");
      Serial1.println(humidityReading);
      Serial1.print("Latitude (degrees*10^-7): ");
      Serial1.println(latitude);
      Serial1.print("Longitude (degrees*10^-7): ");
      Serial1.println(longitude);
      Serial1.print("Altitude (mm): ");
      Serial1.println(altitude);
      Serial1.print("Satelites in view: ");
      Serial1.println(SIV);
      Serial1.print("TVOC (ppb): ");
      Serial1.println(tvoc);
      Serial1.print("Acceleration x,y,z (g): ");
      Serial1.print(gValue.x);
      Serial1.print(", ");
      Serial1.print(gValue.y);
      Serial1.print(", ");
      Serial1.println(gValue.z);
      Serial1.print("Resultant g: ");
      Serial1.println(resultantG);
      Serial1.println("Gyroscope x,y,z (degrees/s): ");
      Serial1.print(gyr.x);
      Serial1.print(", ");
      Serial1.print(gyr.y);
      Serial1.print(", ");
      Serial1.println(gyr.z);
      Serial1.print("Temperature MPU6500 (C): ");
      Serial1.println(temp);
      Serial1.print("Temperature BMP280 (C): ");
      Serial1.println(temp_event.temperature);
      Serial1.print("Pressure (hpa): ");
      Serial1.println(pressure_event.pressure);
      Serial1.print("Heading (degrees): ");
      Serial1.println(mag.HeadingDegress);
      Serial1.print("CO2 (ppb): ");
      Serial1.println(co2concentration);
      Serial1.print("O2 (%): ");
      Serial1.println(o2concentration);

      File file = SD.open("/data.txt", FILE_APPEND);
      file.print("UV index: ");
      file.println(uvIndex);
      file.print("PM1 standarized (µg/m3): ");
      file.println(pms.getPM1_STD());
      file.print("PM1 standarized (µg/m3): ");
      file.println(pms.getPM1_STD());
      file.print("PM2.5 standarized (µg/m3): ");
      file.println(pms.getPM2_5_STD());
      file.print("PM10 standarized (µg/m3): ");
      file.println(pms.getPM10_STD());
      file.print("PM1 atmospheric (µg/m3): ");
      file.println(pms.getPM1_ATM());
      file.print("PM2.5 atmospheric (µg/m3): ");
      file.println(pms.getPM2_5_ATM());
      file.print("PM10 atmospheric (µg/m3): ");
      file.println(pms.getPM10_ATM());
      file.print("PN300 (#/0.1l): ");
      file.println(pms.getCntBeyond300nm());
      file.print("PN500 (#/0.1l): ");
      file.println(pms.getCntBeyond500nm());
      file.print("PN1000 (#/0.1l): ");
      file.println(pms.getCntBeyond1000nm());
      file.print("PN2500 (#/0.1l): ");
      file.println(pms.getCntBeyond2500nm());
      file.print("PN5000 (#/0.1l): ");
      file.println(pms.getCntBeyond5000nm());
      file.print("PN10000 (#/0.1l): ");
      file.println(pms.getCntBeyond10000nm());
      file.print("Temperature DHT11 (C): ");
      file.println(temperatureReading);
      file.print("Humidity (%): ");
      file.println(humidityReading);
      file.print("Latitude (degrees*10^-7): ");
      file.println(latitude);
      file.print("Longitude (degrees*10^-7): ");
      file.println(longitude);
      file.print("Altitude (mm): ");
      file.println(altitude);
      file.print("Satelites in view: ");
      file.println(SIV);
      file.print("TVOC (ppb): ");
      file.println(tvoc);
      file.print("Acceleration x,y,z (g): ");
      file.print(gValue.x);
      file.print(", ");
      file.print(gValue.y);
      file.print(", ");
      file.println(gValue.z);
      file.print("Resultant g: ");
      file.println(resultantG);
      file.println("Gyroscope x,y,z (degrees/s): ");
      file.print(gyr.x);
      file.print(", ");
      file.print(gyr.y);
      file.print(", ");
      file.println(gyr.z);
      file.print("Temperature MPU6500 (C): ");
      file.println(temp);
      file.print("Temperature BMP280 (C): ");
      file.println(temp_event.temperature);
      file.print("Pressure (hpa): ");
      file.println(pressure_event.pressure);
      file.print("Heading (degrees): ");
      file.println(mag.HeadingDegress);
      file.print("CO2 (ppb): ");
      file.println(co2concentration);
      file.print("O2 (%): ");
      file.println(o2concentration);
      file.close();
    }

  }  


  if(setRadioParameters){
    ResponseStructContainer c;
    c = e220ttl.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    configuration.ADDL = radioAddressLo;
    configuration.ADDH = radioAddressHi;
    configuration.CHAN = RadioChannel;
    ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    c = e220ttl.getConfiguration();
    configuration = *(Configuration*) c.data;
    Serial.print(F("AddH: "));  Serial.println(configuration.ADDH, HEX);
	  Serial.print(F("AddL: "));  Serial.println(configuration.ADDL, HEX);
	  Serial.print(F("Chan: "));  Serial.println(configuration.CHAN, DEC);
    c.close();    
  }


}