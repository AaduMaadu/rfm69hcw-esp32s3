/*
  RadioLib RF69 Receive with Interrupts Example

  This example listens for FSK transmissions and tries to
  receive them. Once a packet is received, an interrupt is
  triggered.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#rf69sx1231

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif

/* Uncomment and Comment out for either TX or RX mode*/
//#define TRANSMIT_MODE
#define RECEIVE_MODE

/// RFM69 SPI Configuration
#define RFM69_SCK     7 
#define RFM69_MISO    9 
#define RFM69_MOSI    8 
#define RFM69_CS      6 
#define RFM69_IRQ     4 // G0 Pin in Breakourt board 
#define RFM69_RST     5 
#define RFM69_GPIO    10 // Not currently used 

SPIClass SPI_RF(HSPI);
RF69 radio = new Module(RFM69_CS, RFM69_IRQ, RFM69_RST, RFM69_GPIO, SPI_RF);

// or detect the pinout automatically using RadioBoards
// https://github.com/radiolib-org/RadioBoards
/*
#define RADIO_BOARD_AUTO
#include <RadioBoards.h>
Radio radio = new RadioModule();
*/

#ifdef RECEIVE_MODE
// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // we sent a packet, set the flag
  receivedFlag = true;
}

void setup() {
  Serial.begin(115200);
  // SPI config
  SPI_RF.begin(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CS);

  // initialize RF69 with default settings
  Serial.print(F("[RF69] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // Initialize RFM69 with custom configuration
  // Emsure that it matches with receving/transmitting radio
  radio.setPacketReceivedAction(setFlag);
  radio.setFrequency(434.0);
  static const uint8_t sw[] = {0x10, 0xAF};
  radio.setSyncWord(sw, sizeof(sw));
  radio.variablePacketLengthMode(RADIOLIB_RF69_MAX_PACKET_LENGTH);          
    radio.setBitRate(4.8);            
    radio.setFrequencyDeviation(5);    
    radio.setRxBandwidth(125);      
    radio.setOOK(false);                 
    radio.setOutputPower(20);
    radio.disableAES();
    radio.disableAddressFiltering();
    radio.setCrcFiltering(true);
    radio.setPreambleLength(32);
    radio.setDataShaping(RADIOLIB_SHAPING_0_5);
    radio.setEncoding(RADIOLIB_ENCODING_NRZ);

    Serial.println(F("[RF69] Initialized with configuration"));

  // start listening for packets
  Serial.print(F("[RF69] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // radio.standby()
  // radio.sleep()
  // radio.transmit();
  // radio.receive();
  // radio.readData();
}

// Packet type definitions - must match tracker
#define PKT_TYPE_FC_TELEMETRY   0x01
#define PKT_TYPE_WIFI_RELAY     0x02
#define PKT_TYPE_FC_EVENT       0x03

// FSM state names
const char* fsm_state_name(uint8_t s) {
    switch(s) {
        case 0: return "IDLE";
        case 1: return "ARMED";
        case 2: return "DISARM";
        case 3: return "BURNING";
        case 4: return "RISING";
        case 5: return "APOGEE";
        case 6: return "DROGUE_DESCENT";
        case 7: return "MAIN_DESCENT";
        case 8: return "LANDED";
        default: return "UNKNOWN";
    }
}

// Event type names
const char* event_name(uint8_t t) {
    switch(t) {
        case 0x01: return "BOOT";
        case 0x02: return "ARMED";
        case 0x03: return "DISARMED";
        case 0x04: return "LAUNCH";
        case 0x05: return "BURNOUT";
        case 0x06: return "APOGEE";
        case 0x07: return "DROGUE_FIRED";
        case 0x08: return "MAIN_FIRED";
        case 0x09: return "LANDED";
        case 0x10: return "SENSOR_FAIL";
        case 0x11: return "SD_FAIL";
        case 0xFF: return "GENERIC_ERROR";
        default:   return "UNKNOWN";
    }
}

void decode_packet(uint8_t *buf, int len) {
    if (len < 1) return;
    uint8_t pkt_type = buf[0];

    if (pkt_type == PKT_TYPE_FC_TELEMETRY && len >= 25) {
        // Unpack 24-byte telemetry struct (little-endian)
        uint32_t time_ms;
        int16_t  alt, vel_x10, ax, ay, az, pitch, roll, yaw;
        uint8_t  fsm, flags, pyro;

        memcpy(&time_ms, &buf[1],  4);
        memcpy(&alt,     &buf[5],  2);
        memcpy(&vel_x10, &buf[7],  2);
        memcpy(&ax,      &buf[9],  2);
        memcpy(&ay,      &buf[11], 2);
        memcpy(&az,      &buf[13], 2);
        memcpy(&pitch,   &buf[15], 2);
        memcpy(&roll,    &buf[17], 2);
        memcpy(&yaw,     &buf[19], 2);
        fsm   = buf[21];
        flags = buf[22];
        pyro  = buf[23];

        Serial.println(F("--- FC Telemetry ---"));
        Serial.print(F("  Time:      ")); Serial.print(time_ms); Serial.println(F(" ms"));
        Serial.print(F("  Altitude:  ")); Serial.print(alt);     Serial.println(F(" ft"));
        Serial.print(F("  Vert Vel:  ")); Serial.print(vel_x10 / 10.0f, 1); Serial.println(F(" fps"));
        Serial.print(F("  Accel X:   ")); Serial.print(ax / 1000.0f, 3);    Serial.println(F(" g"));
        Serial.print(F("  Accel Y:   ")); Serial.print(ay / 1000.0f, 3);    Serial.println(F(" g"));
        Serial.print(F("  Accel Z:   ")); Serial.print(az / 1000.0f, 3);    Serial.println(F(" g"));
        Serial.print(F("  Pitch:     ")); Serial.print(pitch);   Serial.println(F(" deg"));
        Serial.print(F("  Roll:      ")); Serial.print(roll);    Serial.println(F(" deg"));
        Serial.print(F("  Yaw:       ")); Serial.print(yaw);     Serial.println(F(" deg"));
        Serial.print(F("  FSM State: ")); Serial.println(fsm_state_name(fsm));
        Serial.print(F("  Flags:     0x")); Serial.println(flags, HEX);
        Serial.print(F("    SD Logging:  ")); Serial.println(flags & 0x01 ? "YES" : "NO");
        Serial.print(F("    Armed:       ")); Serial.println(flags & 0x10 ? "YES" : "NO");
        Serial.print(F("  Pyro Status: 0x")); Serial.println(pyro, HEX);

    } else if (pkt_type == PKT_TYPE_FC_EVENT && len >= 4) {
        Serial.println(F("--- FC Event ---"));
        Serial.print(F("  Event: ")); Serial.println(event_name(buf[2]));
        Serial.print(F("  Data:  0x")); Serial.println(buf[3], HEX);

    } else if (pkt_type == PKT_TYPE_WIFI_RELAY && len >= 2) {
        Serial.println(F("--- WiFi Relay ---"));
        Serial.print(F("  Payload: "));
        for (int i = 1; i < len; i++) {
            if (buf[i] < 0x10) Serial.print('0');
            Serial.print(buf[i], HEX);
            Serial.print(' ');
        }
        Serial.println();

    } else {
        // Unknown type — try to print as APRS/AX.25 (GPS packet)
        // Skip binary AX.25 header, find payload after 0x03 0xF0
        Serial.println(F("--- APRS/GPS ---"));
        for (int i = 0; i < len - 1; i++) {
            if (buf[i] == 0x03 && buf[i+1] == 0xF0) {
                Serial.print(F("  Payload: "));
                for (int j = i + 2; j < len; j++) {
                  Serial.print((char)buf[j]);
                }
                Serial.println();
                return;
            }
        }
        Serial.println(F("  (could not parse)"));
    }
}

void loop() {
  // check if the flag is set
  if(receivedFlag) {
    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    String str;
    uint8_t buf[64];
    int numBytes = radio.getPacketLength();
    int state = radio.readData(buf, numBytes);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int numBytes = radio.getPacketLength();
      int state = radio.readData(byteArr, numBytes);
    */

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[RF69] Received packet!"));

      // print data of the packet
      //Serial.print(F("[RF69] Data:\t\t"));
      //Serial.println(str);

      // print RSSI (Received Signal Strength Indicator)
      // of the last received packet
      Serial.print(F("[RF69] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));
      Serial.print(F("[RF69] Len:  ")); 
      Serial.println(numBytes);
      Serial.print(F("[RF69] Time: \t"));
      Serial.println(millis());
      decode_packet(buf, numBytes);

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            Serial.println(F("[RF69] CRC error"));
    } else {
        Serial.print(F("[RF69] Error code: ")); 
        Serial.println(state);
    }

    // put module back to listen mode
    radio.startReceive();
  }
}

#endif
#ifdef TRANSMIT_MODE
// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

void setup() {
  Serial.begin(115200);
  // SPI config
  SPI_RF.begin(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CS);

  // initialize RF69 with default settings
  Serial.print(F("[RF69] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // set the function that will be called
  // when packet transmission is finished
  radio.setPacketSentAction(setFlag);

  // NOTE: some RF69 modules use high power output,
  //       those are usually marked RF69H(C/CW).
  //       To configure RadioLib for these modules,
  //       you must call setOutputPower() with
  //       second argument set to true.

  Serial.print(F("[RF69] Setting high power module ... "));
  state = radio.setOutputPower(20, true);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }


  // start transmitting the first packet
  Serial.print(F("[RF69] Sending first packet ... "));

  // you can transmit C-string or Arduino string up to
  // 64 characters long
  transmissionState = radio.startTransmit("Hello World!");

  // you can also transmit byte array up to 64 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                      0x89, 0xAB, 0xCD, 0xEF};
    state = radio.startTransmit(byteArr, 8);
  */
}

// counter to keep track of transmitted packets
int count = 0;

void loop() {
  // check if the previous transmission finished
  if(transmittedFlag) {
    // reset flag
    transmittedFlag = false;

    if (transmissionState == RADIOLIB_ERR_NONE) {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()

    } else {
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);

    }

    // clean up after transmission is finished
    // this will ensure transmitter is disabled,
    // RF switch is powered down etc.
    radio.finishTransmit();

    // wait a second before transmitting again
    delay(1000);

    // send another one
    Serial.print(F("[RF69] Sending another packet ... "));

    // you can transmit C-string or Arduino string up to
    // 64 characters long
    String str = "Hello World! #" + String(count++);
    transmissionState = radio.startTransmit(str);

    // you can also transmit byte array up to 64 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
      transmissionState = radio.startTransmit(byteArr, 8);
    */
  }
}
#endif