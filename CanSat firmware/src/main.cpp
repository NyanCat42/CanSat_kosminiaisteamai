#include <Arduino.h>
#include <PMS7003.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
// #include "Adafruit_AGS02MA.h"
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
#define SERVO1PIN 27
#define SERVO2PIN 12
#define SERVO3PIN 14
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
int servoPosition1 = 0;
int servoPosition2 = 20;
int servoPosition3 = 0;

//Humidity sensor defines:

#define DHTTYPE    DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);
int temperatureReading = 0;
int humidityReading = 0;

// GPS defines:

SFE_UBLOX_GNSS myGNSS;
long lastTime  = 0;
long latitude  = 546490000;
long longitude = 250910000;

// // tvoc sensor defines:

// Adafruit_AGS02MA ags;
// uint32_t tvoc = 0;

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
byte radioAddressLo = 0x03;
byte radioAddressHi = 0x00;
int RadioChannel = 23;
int radioAddress = 3;
const int bufferSizeAddCh = 2;
int bufferIndexAddCh = 0;
int AddChBuffer[bufferSizeAddCh];
bool setAddCh = false;


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
float o2concentration = 0;


// Targeting defines

const int numSpoilers = 3;
const int spoilerAngles[numSpoilers] = {0, 120, 240};
int spoilerExtension[numSpoilers];
int servoAngle = 180;
int calculatedHeading = 0;
double canSatHeading = 0;
double latitudeTarget = 551142827 / 10000000.0;
double longitudeTarget = 253362947 / 10000000.0;
bool servoTargeting = false;


void setup() {

  Serial.begin(115200);

  // Radio setup

	e220ttl.begin();
    ResponseStructContainer c;
    c = e220ttl.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;

    configuration.ADDL = 5;
    configuration.ADDH = 1;
    configuration.CHAN = 25;

    configuration.SPED.uartBaudRate = UART_BPS_9600; // Serial baud rate
    configuration.SPED.airDataRate = AIR_DATA_RATE_100_96; // Air baud rate
    configuration.SPED.uartParity = MODE_00_8N1; // Parity bit

    configuration.OPTION.subPacketSetting = SPS_200_00; // Packet size
    configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED; // Need to send special command
    configuration.OPTION.transmissionPower = POWER_22; // Device power

    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED; // Enable RSSI info
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION; // Enable repeater mode
    configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED; // Check interference
    configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011; // WOR timing
    ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    c = e220ttl.getConfiguration();
    configuration = *(Configuration*) c.data;
    
    Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
    Serial.println(F(" "));
    Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
    Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
    Serial.println(F(" "));
    Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -&gt; "); Serial.println(configuration.getChannelDescription());
    Serial.println(F(" "));
    Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -&gt; "); Serial.println(configuration.SPED.getUARTParityDescription());
    Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -&gt; "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
    Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -&gt; "); Serial.println(configuration.SPED.getAirDataRateDescription());
    Serial.println(F(" "));
    Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -&gt; "); Serial.println(configuration.OPTION.getSubPacketSetting());
    Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -&gt; "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
    Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -&gt; "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
    Serial.println(F(" "));
    Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
    Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
    Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
    Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());

    c.close();    

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


  // // TVOC sensor setup:

  // if (! ags.begin(&Wire, 0x1A)) {
  //   Serial.println("Couldn't find AGS20MA sensor, check your wiring and pullup resistors!");
  //   while (1) yield();
  // }

  // if (ags.getFirmwareVersion() == 0) {
  //   Serial.println(F("Could not read firmware, I2C communications issue?"));
  //   while (1) yield();
  // }

  // Serial.print("Firmware version: 0x");
  // Serial.println(ags.getFirmwareVersion(), HEX);
  // ags.printSensorDetails();

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

// SPI.begin(18, 23, 19, 5);
// SPI.setDataMode(SPI_MODE0);

//     if(!SD.begin(5)){
//         Serial.println("Card Mount Failed");
//         return;
//     }
//     uint8_t cardType = SD.cardType();

//     if(cardType == CARD_NONE){
//         Serial.println("No SD card attached");
//         return;
//     }

//     Serial.print("SD Card Type: ");
//     if(cardType == CARD_MMC){
//         Serial.println("MMC");
//     } else if(cardType == CARD_SD){
//         Serial.println("SDSC");
//     } else if(cardType == CARD_SDHC){
//         Serial.println("SDHC");
//     } else {
//         Serial.println("UNKNOWN");
//     }

// O2 sensor setup:

  o2Serial.begin(9600, SWSERIAL_8N1, O2RX, -1, false);
  o2Serial.enableIntTx(false);

// CO2 sensor setup:

  co2Serial.begin(9600, SWSERIAL_8N1, CO2RX, -1, false);
  co2Serial.enableIntTx(false);

}

void loop() {

    // Magnetometer read:
    float declinationAngle = (8.0 + (35.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);
    sVector_t mag = compass.readRaw();
    compass.getHeadingDegrees();
    canSatHeading = mag.HeadingDegress;

    // Targeted landing
    if(servoTargeting){
      double latitudeDouble = latitude / 10000000.0;
      double longitudeDouble = longitude / 10000000.0;

      double dLon = (longitudeTarget - longitudeDouble) * DEG_TO_RAD;
      double lat1 = latitudeDouble * DEG_TO_RAD;
      double lat2 = latitudeTarget * DEG_TO_RAD;
      double y = sin(dLon) * cos(lat2);
      double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
      double heading = atan2(y, x) * RAD_TO_DEG;

      if (heading < 0) {
        heading += 360.0;
      }

      calculatedHeading = heading - canSatHeading;

        if (calculatedHeading < 0) {
        calculatedHeading += 360.0;
      } else if (calculatedHeading > 360.0) {
        calculatedHeading -= 360.0;
      }

      for (int i = 0; i < numSpoilers; i++) {
        int angleDifference = (calculatedHeading - spoilerAngles[i] + 360) % 360;
        int diff = min(abs(angleDifference), 360 - abs(angleDifference));
        spoilerExtension[i] = servoAngle - (diff / 120.0 * servoAngle);
      }

      servoPosition1 = map(spoilerExtension[0], -90, 180, 150, 0);
      servoPosition2 = map(spoilerExtension[1], -90, 180, 170, 20);
      servoPosition3 = map(spoilerExtension[2], -90, 180, 150, 0);

    }

    
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
        o2concentration = (hiByteO2 * 256 + loByteO2) / 10.00;
        bufferIndexO2 = 0;
        found0xFF = false;
      }
    }
  }


  // set adress, channel
  if (e220ttl.available()>1) {
    ResponseContainer rc = e220ttl.receiveMessage();
    int incomingCommand = rc.data.toInt();
    if (incomingCommand == 1) {
      setAddCh = true;
      Serial.print(F("Adress -> Enter; Channel -> Enter"));
    } else if (setAddCh) {
      AddChBuffer[bufferIndexAddCh++] = incomingCommand;
      if (bufferIndexAddCh == 2) {
        radioAddress = AddChBuffer[0];
        RadioChannel = AddChBuffer[1];
        radioAddressHi = (radioAddress >> 8) & 0xFF;
        radioAddressLo = radioAddress & 0xFF;
        bufferIndexAddCh = 0;
        setAddCh = false;

        ResponseStructContainer c;
        c = e220ttl.getConfiguration();
        Configuration configuration = *(Configuration*) c.data;
        configuration.ADDL = radioAddressLo;
        configuration.ADDH = radioAddressHi;
        configuration.CHAN = RadioChannel;

        configuration.SPED.uartBaudRate = UART_BPS_9600; // Serial baud rate
        configuration.SPED.airDataRate = AIR_DATA_RATE_100_96; // Air baud rate
        configuration.SPED.uartParity = MODE_00_8N1; // Parity bit

        configuration.OPTION.subPacketSetting = SPS_200_00; // Packet size
        configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED; // Need to send special command
        configuration.OPTION.transmissionPower = POWER_22; // Device power

        configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED; // Enable RSSI info
        configuration.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION; // Enable repeater mode
        configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED; // Check interference
        configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011; // WOR timing
        ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
        c = e220ttl.getConfiguration();
        configuration = *(Configuration*) c.data;
        Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
        Serial.println(F(" "));
        Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
        Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
        Serial.println(F(" "));
        Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -&gt; "); Serial.println(configuration.getChannelDescription());
        Serial.println(F(" "));
        Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -&gt; "); Serial.println(configuration.SPED.getUARTParityDescription());
        Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -&gt; "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
        Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -&gt; "); Serial.println(configuration.SPED.getAirDataRateDescription());
        Serial.println(F(" "));
        Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -&gt; "); Serial.println(configuration.OPTION.getSubPacketSetting());
        Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -&gt; "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
        Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -&gt; "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
        Serial.println(F(" "));
        Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
        Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
        Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
        Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());
        c.close();    

      }
    } else if (incomingCommand == 2 && setAddCh == false){
        Serial.print("Opening sides");
        servoPosition1 = 0;
        servoPosition2 = 20;
        servoPosition3 = 0;
        servoTargeting = false;
    } else if (incomingCommand == 3 && setAddCh == false){
        Serial.print("Closing sides");
        servoPosition1 = 150;
        servoPosition2 = 180;
        servoPosition3 = 150;
        servoTargeting = false;
    } else if (incomingCommand == 4 && setAddCh == false){
        servoTargeting = true;
    }
  }

  servo1.write(servoPosition1);
  servo2.write(servoPosition2);
  servo3.write(servoPosition3);

  // if (millis() - lastTime4 > 10000)
  // {
  //   lastTime4 = millis(); //Update the timer

  //   digitalWrite(BEEPER, HIGH);
  //   delay(1);
  //   digitalWrite(BEEPER, LOW);

  // }


  // Read sensors
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer

    // UV sensoe read:
    int uvIndex = analogRead(13);

    // Particle sensor read:
    pms.poll();
    pms.read();
    
    // GPS read:
    latitude = myGNSS.getLatitude();
    longitude = myGNSS.getLongitude();
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

    // // Read veery slow sensor
    // if (millis() - lastTime2 > 10000)
    // {
    //   lastTime2 = millis(); //Update the timer
    //   // TVOC sensor read:
    //   tvoc = ags.getTVOC();
    // }


      // if (millis() - lastTime3 > 10000) {
      //   lastTime3 = millis(); //Update the timer
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
      // Serial.print("TVOC (ppb): ");
      // Serial.println(tvoc);
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

      Serial2.print("/*EMA,"); // 1
      Serial2.print(pms.getPM1_STD()); //2
      Serial2.print(",");
      Serial2.print(pms.getPM2_5_STD()); //3
      Serial2.print(",");
      Serial2.print(pms.getPM10_STD()); //4
      Serial2.print(",");
      Serial2.print(pms.getPM1_ATM()); // 5
      Serial2.print(",");
      Serial2.print(pms.getPM2_5_ATM()); //6
      Serial2.print(",");
      Serial2.print(pms.getPM10_ATM());
      Serial2.print(",");
      Serial2.print(pms.getCntBeyond300nm());
      Serial2.print(",");
      Serial2.print(pms.getCntBeyond500nm());
      Serial2.print(",");
      Serial2.print(pms.getCntBeyond1000nm());
      Serial2.print(",");
      Serial2.print(pms.getCntBeyond2500nm());
      Serial2.print(",");
      Serial2.print(pms.getCntBeyond5000nm());
      Serial2.print(",");
      Serial2.print(pms.getCntBeyond10000nm());
      Serial2.print(",");
      float printLat = latitude/10000000.00000000;
      Serial2.print(printLat, 7);
      Serial2.print(",");
      float printLong = longitude/10000000.00000000;
      Serial2.print(printLong, 7);
      Serial2.print(",");
      Serial2.print(altitude/1000);
      Serial2.print(",");
      Serial2.print(SIV);
      Serial2.print(",");
      Serial2.print(gValue.x);
      Serial2.print(",");
      Serial2.print(gValue.y);
      Serial2.print(",");
      Serial2.print(gValue.z);
      Serial2.print(",");
      Serial2.print(gyr.x);
      Serial2.print(",");
      Serial2.print(gyr.y);
      Serial2.print(",");
      Serial2.print(gyr.z);
      Serial2.print(",");
      Serial2.print(temperatureReading);
      Serial2.print(",");
      Serial2.print(temp);
      Serial2.print(",");
      Serial2.print(temp_event.temperature);
      Serial2.print(",");
      Serial2.print(co2concentration);
      Serial2.print(",");
      Serial2.print(o2concentration, 1);
      Serial2.print(",");
      Serial2.print(humidityReading);
      Serial2.print(",");
      Serial2.print(pressure_event.pressure);
      Serial2.print(",");
      Serial2.print(mag.HeadingDegress);
      Serial2.print(",");
      Serial2.print(uvIndex);
      Serial2.println("*/");




    //   File file = SD.open("/data.txt", FILE_APPEND);
    //   file.print("UV index: ");
    //   file.println(uvIndex);
    //   file.print("PM1 standarized (µg/m3): ");
    //   file.println(pms.getPM1_STD());
    //   file.print("PM1 standarized (µg/m3): ");
    //   file.println(pms.getPM1_STD());
    //   file.print("PM2.5 standarized (µg/m3): ");
    //   file.println(pms.getPM2_5_STD());
    //   file.print("PM10 standarized (µg/m3): ");
    //   file.println(pms.getPM10_STD());
    //   file.print("PM1 atmospheric (µg/m3): ");
    //   file.println(pms.getPM1_ATM());
    //   file.print("PM2.5 atmospheric (µg/m3): ");
    //   file.println(pms.getPM2_5_ATM());
    //   file.print("PM10 atmospheric (µg/m3): ");
    //   file.println(pms.getPM10_ATM());
    //   file.print("PN300 (#/0.1l): ");
    //   file.println(pms.getCntBeyond300nm());
    //   file.print("PN500 (#/0.1l): ");
    //   file.println(pms.getCntBeyond500nm());
    //   file.print("PN1000 (#/0.1l): ");
    //   file.println(pms.getCntBeyond1000nm());
    //   file.print("PN2500 (#/0.1l): ");
    //   file.println(pms.getCntBeyond2500nm());
    //   file.print("PN5000 (#/0.1l): ");
    //   file.println(pms.getCntBeyond5000nm());
    //   file.print("PN10000 (#/0.1l): ");
    //   file.println(pms.getCntBeyond10000nm());
    //   file.print("Temperature DHT11 (C): ");
    //   file.println(temperatureReading);
    //   file.print("Humidity (%): ");
    //   file.println(humidityReading);
    //   file.print("Latitude (degrees*10^-7): ");
    //   file.println(latitude);
    //   file.print("Longitude (degrees*10^-7): ");
    //   file.println(longitude);
    //   file.print("Altitude (mm): ");
    //   file.println(altitude);
    //   file.print("Satelites in view: ");
    //   file.println(SIV);
    //   // file.print("TVOC (ppb): ");
    //   // file.println(tvoc);
    //   file.print("Acceleration x,y,z (g): ");
    //   file.print(gValue.x);
    //   file.print(", ");
    //   file.print(gValue.y);
    //   file.print(", ");
    //   file.println(gValue.z);
    //   file.print("Resultant g: ");
    //   file.println(resultantG);
    //   file.println("Gyroscope x,y,z (degrees/s): ");
    //   file.print(gyr.x);
    //   file.print(", ");
    //   file.print(gyr.y);
    //   file.print(", ");
    //   file.println(gyr.z);
    //   file.print("Temperature MPU6500 (C): ");
    //   file.println(temp);
    //   file.print("Temperature BMP280 (C): ");
    //   file.println(temp_event.temperature);
    //   file.print("Pressure (hpa): ");
    //   file.println(pressure_event.pressure);
    //   file.print("Heading (degrees): ");
    //   file.println(mag.HeadingDegress);
    //   file.print("CO2 (ppb): ");
    //   file.println(co2concentration);
    //   file.print("O2 (%): ");
    //   file.println(o2concentration);
    //   file.close();
    //}

    // digitalWrite(BEEPER, HIGH);
    // delay(1);
    // digitalWrite(BEEPER, LOW);

  }  


}