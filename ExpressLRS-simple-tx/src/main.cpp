#include <Arduino.h>
#if defined(PLATFORM_LORA32U4)
#include <RadioLib.h>
#elif defined(PLATFORM_CUBECELL)
#include <radio/radio.h>
#endif

#define RXBuffSize 8
uint8_t RXdataBuffer[RXBuffSize];
 
#define FREQ_TIME_MS 10000
#define FREQ_COUNT 40
uint32_t _freqExpire = 0;
uint8_t _freqIndex = 0;

#if defined(PLATFORM_LORA32U4)
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
#elif defined(PLATFORM_CUBECELL)
const uint32_t FHSSfreqs[] = {
  903500000,
  904100000,
  904700000,
  905300000,

  905900000,
  906500000,
  907100000,
  907700000,

  908300000,
  908900000,
  909500000,
  910100000,

  910700000,
  911300000,
  911900000,
  912500000,

  913100000,
  913700000,
  914300000,
  914900000,

  915500000,
  916100000,
  916700000,
  917300000,

  917900000,
  918500000,
  919100000,
  919700000,

  920300000,
  920900000,
  921500000,
  922100000,

  922700000,
  923300000,
  923900000,
  924500000,

  925100000,
  925700000,
  926300000,
  926900000
};
#endif

#if defined(PLATFORM_LORA32U4)
// SX1276 has the following connections:
// NSS pin:   8
// DIO0 pin:  7
// RESET pin: 4
// DIO1 pin:  NC
SX1276 radio = new Module(8, 7, 4, RADIOLIB_NC);
#elif defined(PLATFORM_CUBECELL)
RadioEvents_t RadioEvents;
#endif

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

#if defined(PLATFORM_LORA32U4)
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
#elif defined(PLATFORM_CUBECELL)
int16_t RXrssi;
int8_t RXsnr;

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  memcpy(RXdataBuffer, payload, size );
  RXrssi=rssi;
  RXsnr=snr;

  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}

bool errorFlag = false;

void OnRxError()
{
  errorFlag = true;
}

#endif

void setup() {
  while(!Serial);
  Serial.begin(115200);

  // initialize SX1278 with default settings
  Serial.print(F("[SX12xx] Initializing ... "));

#if defined(PLATFORM_LORA32U4)
  int state = radio.begin(FHSSfreqs[_freqIndex], 500.0, 6, 7);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  //radio.invertIQ(true);

  // set the function that will be called
  // when new packet is received
  radio.setDio0Action(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX1276] Starting to listen ... "));
  state = radio.startReceive(RXBuffSize);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
#elif defined(PLATFORM_CUBECELL)
#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       6         // [SF7..SF12]
#define LORA_CODINGRATE                             3         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  true
#define LORA_IQ_INVERSION_ON                        false

RadioEvents.RxDone = OnRxDone;
RadioEvents.RxError = OnRxError;
Radio.Init( &RadioEvents );
Radio.SetChannel( FHSSfreqs[_freqIndex] );
	
Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                   RXBuffSize, true, 0, 0, LORA_IQ_INVERSION_ON, true );

Serial.println(F("success!"));

Radio.Rx( 0 );

Serial.println(F("[SX1262] into RX mode"));
#endif 
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

void printData()
{
      // packet was successfully received
      Serial.println(F("[SX12xx] Received packet!"));

      Serial.println(FHSSfreqs[_freqIndex]);

      // print data of the packet
      Serial.print(F("[SX12xx] Data:\t\t"));
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
        Serial.println();
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
}


void loop() {
  // check if the flag is set
  if(receivedFlag) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;

#if defined(PLATFORM_LORA32U4)
    // you can also read received data as byte array
    int state = radio.readData(RXdataBuffer, RXBuffSize);
    
    if (state == ERR_NONE) {

      printData();
      
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
    radio.startReceive(RXBuffSize);
#elif defined(PLATFORM_CUBECELL)
    printData();
    
    // print RSSI (Received Signal Strength Indicator)
    Serial.print(F("[SX1262] RSSI:\t\t"));
    Serial.print(RXrssi);
    Serial.println(F(" dBm"));

    // print SNR (Signal-to-Noise Ratio)
    Serial.print(F("[SX1262] SNR:\t\t"));
    Serial.print(RXsnr);
    Serial.println(F(" dB"));

    Radio.Rx( 0 );
#endif

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }

#if defined(PLATFORM_CUBECELL)
  else if (errorFlag)
  {
    enableInterrupt = false;

    // reset flag
    errorFlag = false;

    // packet was received, but is malformed
    Serial.println(F("[SX1262] error!"));

    Radio.Rx( 0 );

    enableInterrupt = true;
  }
#endif

  // Change frequency if it is time
  if (millis() > _freqExpire)
  {
    _freqExpire = millis() + FREQ_TIME_MS;
    _freqIndex++;

    if (_freqIndex == FREQ_COUNT)
    {
        _freqIndex = 0;
    }

#if defined(PLATFORM_LORA32U4)
    radio.standby();
    radio.setFrequency(FHSSfreqs[_freqIndex]);
    radio.startReceive(RXBuffSize);
#elif defined(PLATFORM_CUBECELL)
    Radio.Standby();
    Radio.SetChannel(FHSSfreqs[_freqIndex]);
    Radio.Rx( 0 );
#endif
  }

#if defined(PLATFORM_CUBECELL)
    Radio.IrqProcess();
#endif
}
