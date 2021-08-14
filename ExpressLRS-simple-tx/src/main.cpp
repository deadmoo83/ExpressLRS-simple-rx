/*
   RadioLib SX127x Channel Activity Detection Example
   This example scans the current LoRa channel and detects
   valid LoRa preambles. Preamble is the first part of
   LoRa transmission, so this can be used to check
   if the LoRa channel is free, or if you should start
   receiving a message.
   Other modules from SX127x/RFM9x family can also be used.
   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem
   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

#define TXRXBuffSize 16
uint8_t RXdataBuffer[TXRXBuffSize];
 
#define FREQ_TIME_MS 5000
#define FREQ_COUNT 40
uint32_t _freqExpire = 0;
uint8_t _freqIndex = 0;
const float FHSSfreqs[] = {
    903.500000,
    904.100000,
    904.700000,
    905.300000,

    905.900000,
    906.500000,
    907.100000,
    907.700000,

    908.300000,
    908.900000,
    909.500000,
    910.100000,

    910.700000,
    911.300000,
    911.900000,
    912.500000,

    913.100000,
    913.700000,
    914.300000,
    914.900000,

    915.500000,
    916.100000,
    916.700000,
    917.300000,

    917.900000,
    918.500000,
    919.100000,
    919.700000,

    920.300000,
    920.900000,
    921.500000,
    922.100000,

    922.700000,
    923.300000,
    923.900000,
    924.500000,

    925.100000,
    925.700000,
    926.300000,
    926.900000
};

// SX1276 has the following connections:
// NSS pin:   8
// DIO0 pin:  7
// RESET pin: 4
// DIO1 pin:  NC
SX1276 radio = new Module(8, 7, 4, RADIOLIB_NC);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1276 radio = RadioShield.ModuleA;

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}

void setup() {
  while(!Serial);
  Serial.begin(115200);

  // initialize SX1278 with default settings
  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin(FHSSfreqs[_freqIndex], 500.0, 6, 7);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called
  // when new packet is received
  radio.setDio0Action(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX1276] Starting to listen ... "));
  state = radio.startReceive(TXRXBuffSize);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
}

#define RC_DATA_PACKET 0b00
#define MSP_DATA_PACKET 0b01
#define TLM_PACKET 0b11
#define SYNC_PACKET 0b10

#define PACKED __attribute__((packed))

/**
 * Crossfire packed channel structure, each channel is 11 bits
 */
typedef struct crsf_channels_s
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED crsf_channels_t;

static volatile crsf_channels_s PackedRCdataOut;            // RC data in packed format for output.

#define CRSF_CHANNEL_VALUE_1000 191
#define CRSF_CHANNEL_VALUE_MID  992
#define CRSF_CHANNEL_VALUE_2000 1792

// Convert a bit into either the CRSF value for 1000 or 2000
static inline uint16_t BIT_to_CRSF(uint8_t val)
{
    return (val) ? CRSF_CHANNEL_VALUE_2000 : CRSF_CHANNEL_VALUE_1000;
}

// 3b switches use 0-5 to represent 6 positions switches and "7" to represent middle
// The calculation is a bit non-linear all the way to the endpoints due to where
// Ardupilot defines its modes
static inline uint16_t SWITCH3b_to_CRSF(uint16_t val)
{
  switch (val)
  {
  case 0: return CRSF_CHANNEL_VALUE_1000;
  case 5: return CRSF_CHANNEL_VALUE_2000;
  case 6: // fallthrough
  case 7: return CRSF_CHANNEL_VALUE_MID;
  default: // (val - 1) * 240 + 630; aka 150us spacing, starting at 1275
    return val * 240 + 391;
  }
}

// Convert 0-max to the CRSF values for 1000-2000
static inline uint16_t N_to_CRSF(uint16_t val, uint16_t max)
{
   return val * (CRSF_CHANNEL_VALUE_2000-CRSF_CHANNEL_VALUE_1000) / max + CRSF_CHANNEL_VALUE_1000;
}

/**
 * Hybrid switches decoding of over the air data
 *
 * Hybrid switches uses 10 bits for each analog channel,
 * 2 bits for the low latency switch[0]
 * 3 bits for the round-robin switch index and 2 bits for the value
 *
 * Input: Buffer
 * Output: crsf->PackedRCdataOut
 */
void UnpackChannelDataHybridSwitch8()
{
    // The analog channels
    PackedRCdataOut.ch0 = (RXdataBuffer[1] << 3) | ((RXdataBuffer[5] & 0b11000000) >> 5);
    PackedRCdataOut.ch1 = (RXdataBuffer[2] << 3) | ((RXdataBuffer[5] & 0b00110000) >> 3);
    PackedRCdataOut.ch2 = (RXdataBuffer[3] << 3) | ((RXdataBuffer[5] & 0b00001100) >> 1);
    PackedRCdataOut.ch3 = (RXdataBuffer[4] << 3) | ((RXdataBuffer[5] & 0b00000011) << 1);

    // The low latency switch
    PackedRCdataOut.ch4 = BIT_to_CRSF((RXdataBuffer[6] & 0b01000000) >> 6);

    // The round-robin switch, switchIndex is actually index-1
    // to leave the low bit open for switch 7 (sent as 0b11x)
    // where x is the high bit of switch 7
    uint8_t switchIndex = (RXdataBuffer[6] & 0b111000) >> 3;
    uint16_t switchValue = SWITCH3b_to_CRSF(RXdataBuffer[6] & 0b111);

    switch (switchIndex) {
        case 0:
            PackedRCdataOut.ch5 = switchValue;
            break;
        case 1:
            PackedRCdataOut.ch6 = switchValue;
            break;
        case 2:
            PackedRCdataOut.ch7 = switchValue;
            break;
        case 3:
            PackedRCdataOut.ch8 = switchValue;
            break;
        case 4:
            PackedRCdataOut.ch9 = switchValue;
            break;
        case 5:
            PackedRCdataOut.ch10 = switchValue;
            break;
        case 6:   // Because AUX1 (index 0) is the low latency switch, the low bit
        case 7:   // of the switchIndex can be used as data, and arrives as index "6"
            PackedRCdataOut.ch11 = N_to_CRSF(RXdataBuffer[6] & 0b1111, 15);
            break;
    }
}

void loop() {
  // check if the flag is set
  if(receivedFlag) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;

    // you can also read received data as byte array
    int state = radio.readData(RXdataBuffer, TXRXBuffSize);
    
    if (state == ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[SX1276] Received packet!"));

      Serial.println(FHSSfreqs[_freqIndex]);

      // print data of the packet
      Serial.print(F("[SX1276] Data:\t\t"));
      uint8_t type = RXdataBuffer[0] & 0b11;
      switch(type)
      {
      case RC_DATA_PACKET:
        Serial.println(F("RC_DATA_PACKET"));
        UnpackChannelDataHybridSwitch8();
        Serial.print(PackedRCdataOut.ch0);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch1);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch2);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch3);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch4);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch5);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch6);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch7);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch8);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch9);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch10);
        Serial.print(F(", "));
        Serial.print(PackedRCdataOut.ch11);
        Serial.print(F(", "));
        break;
      case MSP_DATA_PACKET:
        Serial.println(F("MSP_DATA_PACKET"));
        break;
      case TLM_PACKET:
        Serial.println(F("TLM_PACKET"));
        break;
      case SYNC_PACKET:
        Serial.println(F("SYNC_PACKET"));
        break;
      default:
        Serial.println(F("UNKNOWN"));
      }
 
      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX1276] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1276] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F("[SX1276] Frequency error:\t"));
      Serial.print(radio.getFrequencyError());
      Serial.println(F(" Hz"));

    } else if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[SX1276] CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("[SX1276] Failed, code "));
      Serial.println(state);

    }

    // put module back to listen mode
    radio.startReceive(TXRXBuffSize);

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }

  // Change frequency if it is time
  if (millis() > _freqExpire)
  {

    radio.standby();

    _freqExpire = millis() + FREQ_TIME_MS;
    _freqIndex++;

    if (_freqIndex == FREQ_COUNT)
    {
        _freqIndex = 0;
    }

    radio.setFrequency(FHSSfreqs[_freqIndex]);
    radio.startReceive(TXRXBuffSize);
  }
}
