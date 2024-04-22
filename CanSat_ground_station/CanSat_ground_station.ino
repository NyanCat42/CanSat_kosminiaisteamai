#include "Arduino.h"
#include "LoRa_E220.h"

// ---------- Arduino pins --------------
LoRa_E220 e220ttl(4, 5, 3, 7, 6); // Arduino RX <-- e220 TX, Arduino TX --> e220 RX AUX M0 M1

byte radioAddressLo = 0x03;
byte radioAddressHi = 0x00;
int RadioChannel = 23;
int radioAddress = 3;
const int bufferSizeAddCh = 2;
int bufferIndexAddCh = 0;
int AddChBuffer[bufferSizeAddCh];
bool setAddCh = false;

void setup() {
	Serial.begin(9600);
	while(!Serial){};
	delay(500);

	Serial.println();


	// Startup all pins and UART
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
}

void loop() {

  if (e220ttl.available()>1) {
    ResponseContainer rc = e220ttl.receiveMessage();
    Serial.print(rc.data);
  }

  if (Serial.available()) {
	  String input = Serial.readString();
	  e220ttl.sendMessage(input);

    int incomingCommand = input.toInt();
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
    }

  }

}