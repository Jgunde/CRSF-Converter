// This script reads in the CRSF receiver protocol and outputs the PPM protocol.
// It also reads in the Lightweight Telemetry (LTM) protocol and sends the telemetry back to the CRSF receiver.

// Code was drawn from these repos:
// CRSF: https://github.com/CapnBry/CRServoF/tree/master/lib
// PPM: https://quadmeup.com/generate-ppm-signal-with-arduino/
//   or here: https://github.com/DzikuVx/ppm_encoder
// LTM: https://github.com/DzikuVx/ltm_telemetry_reader


#include <SoftwareSerial.h>
#include "CrsfSerial.h"


////////////////////// PPM CONFIGURATION //////////////////////

#define CHANNEL_NUMBER 12  //set the number of channels
#define CHANNEL_DEFAULT_VALUE 1000  //set the default servo value
#define CHANNEL_FAILSAFE_VALUE 0  // failsafe mode is "no pules"
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10  //set PPM signal output pin on the arduino

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[CHANNEL_NUMBER];

ISR(TIMER1_COMPA_vect) {  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

//////////////////////////////////////////////////////////////////



////////////////////// CRSF CONFIGURATION //////////////////////

// NOTE:
//    I chose 115200 for the baud rate because it's more ubiquitous than 250000.
//    Also, since PPM is a slow protocol, I don't think it'll even matter.

// Pass any HardwareSerial port
// "Arduino" users (atmega328) can not use CRSF_BAUDRATE, as the atmega does not support it
// and should pass 250000, but then also must flash the receiver with RCVR_UART_BAUD=250000
// Also note the atmega only has one Serial, so logging to Serial must be removed
CrsfSerial crsf(Serial, 115200);

//////////////////////////////////////////////////////////////////



////////////////////// LTM CONFIGURATION //////////////////////

SoftwareSerial ltmSerial(11, 12);

String fixTypes[3] = {
  "NO",
  "2D",
  "3D"
};

enum ltmStates {
  IDLE,
  HEADER_START1,
  HEADER_START2,
  HEADER_MSGTYPE,
  HEADER_DATA
};

#define LONGEST_FRAME_LENGTH 18

#define GFRAMELENGTH 18
#define AFRAMELENGTH 10
#define SFRAMELENGTH 11
#define OFRAMELENGTH 18
#define NFRAMELENGTH 10
#define XFRAMELENGTH 10

const char* flightModes[] = {
  "Manual",
  "Rate",
  "Angle",
  "Horizon",
  "Acro",
  "Stabilized1",
  "Stabilized2",
  "Stabilized3",
  "Altitude Hold",
  "GPS Hold",
  "Waypoints",
  "Head free",
  "Circle",
  "RTH",
  "Follow me",
  "Land",
  "Fly by wire A",
  "Fly by wire B",
  "Cruise",
  "Unknown"
};

typedef struct remoteData_s {
  int pitch;
  int roll;
  int heading;
  uint16_t voltage;
  byte rssi;
  bool armed;
  bool failsafe;
  byte flightmode;

  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  uint8_t groundSpeed; 
  int16_t hdop;
  uint8_t gpsFix;
  uint8_t gpsSats;

  int32_t homeLatitude;
  int32_t homeLongitude;

  uint8_t sensorStatus;
} remoteData_t;

remoteData_t remoteData;

uint8_t serialBuffer[LONGEST_FRAME_LENGTH];
uint8_t state = IDLE;
char frameType;
byte frameLength;
byte receiverIndex;

byte readByte(uint8_t offset) {
  return serialBuffer[offset];
}

int readInt(uint8_t offset) {
  return (int) serialBuffer[offset] + ((int) serialBuffer[offset + 1] << 8);
}

int32_t readInt32(uint8_t offset) {
  return (int32_t) serialBuffer[offset] + ((int32_t) serialBuffer[offset + 1] << 8) + ((int32_t) serialBuffer[offset + 2] << 16) + ((int32_t) serialBuffer[offset + 3] << 24);
}

void checkLTM() {
  if (ltmSerial.available()) {

    char data = ltmSerial.read();

    if (state == IDLE) {
      if (data == '$') {
        state = HEADER_START1;
      }
    } else if (state == HEADER_START1) {
      if (data == 'T') {
        state = HEADER_START2;
      } else {
        state = IDLE;
      }
    } else if (state == HEADER_START2) {
      frameType = data;
      state = HEADER_MSGTYPE;
      receiverIndex = 0;

      switch (data) {

        case 'G':
          frameLength = GFRAMELENGTH;
          break;
        case 'A':
          frameLength = AFRAMELENGTH;
          break;
        case 'S':
          frameLength = SFRAMELENGTH;
          break;
        case 'O':
          frameLength = OFRAMELENGTH;
          break;
        case 'N':
          frameLength = NFRAMELENGTH;
          break;
        case 'X':
          frameLength = XFRAMELENGTH;
          break;
        default:
          state = IDLE;
      }

    } else if (state == HEADER_MSGTYPE) {

      /*
       * Check if last payload byte has been received.
       */
      if (receiverIndex == frameLength - 4) {
        /*
         * If YES, check checksum and execute data processing
         */

        if (frameType == 'A') {
            remoteData.pitch = readInt(0);
            remoteData.roll = readInt(2);
            remoteData.heading = readInt(4);

            /////////////////////////

            // LTM
            // Pitch - int16, degrees
            // Roll	- int16, degrees
            // Heading - int16, degrees. Course over ground

            // NOTE: Heading is sent with the GPS data (in frame G)
        }

        if (frameType == 'S') {
            remoteData.voltage = readInt(0);
            remoteData.rssi = readByte(4);

            byte raw = readByte(6);
            remoteData.flightmode = raw >> 2;

            /////////////////////////

            // LTM
            // Vbat	- uint16, mV
            // Battery Consumption - uint16, mAh
            // RSSI -	uchar
            // Airspeed	- uchar, m/s
            // Status	- uchar

            // CRSF
            // unsigned voltage : 16;  // V * 10 big endian
            // unsigned current : 16;  // A * 10 big endian
            // unsigned capacity : 24; // mah big endian
            // unsigned remaining : 8; // %

            crsf_sensor_battery_t crsf_batt = { 0 };
            crsf_batt.voltage = htobe16(uint16_t(remoteData.voltage / 100));  // (Values are MSB first (BigEndian) so we must swap the order.)
            crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsf_batt, sizeof(crsf_batt));  // TODO: Use CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE?

            /////////////////////////

            crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_FLIGHT_MODE, &remoteData.flightmode, sizeof(remoteData.flightmode));  // TODO: I don't think this is properly sending the flight mode. I think we need to use the strings in the flightModes array.

        }

        if (frameType == 'G') {
            remoteData.latitude = readInt32(0);
            remoteData.longitude = readInt32(4);
            remoteData.groundSpeed = readByte(8);
            remoteData.altitude = readInt32(9);

            uint8_t raw = readByte(13);
            remoteData.gpsSats = raw >> 2;
            remoteData.gpsFix = raw & 0x03;

            /////////////////////////

            // LTM
            // Latitude	- int32 decimal degrees * 10,000,000 (1E7)
            // Longitude - int32 decimal degrees * 10,000,000 (1E7)
            // Ground Speed	- uchar m/s
            // Altitude	- (u)int32, cm (m / 100). In the original specification, this was unsigned. In iNav it is signed and should be so interpreted by consumers
            // Sats	- uchar. bits 0-1 : fix ; bits 2-7 : number of satellites

            // CRSF
            // int32_t latitude;      // degree / 10,000,000 big endian
            // int32_t longitude;     // degree / 10,000,000 big endian
            // uint16_t groundspeed;  // km/h / 10 big endian
            // uint16_t heading;      // GPS heading, degree/100 big endian
            // uint16_t altitude;     // meters, +1000m big endian
            // uint8_t satellites;    // satellites

            crsf_sensor_gps_t crsf_gps = { 0 };

            crsf_gps.latitude = htobe32(uint32_t(remoteData.latitude));
            crsf_gps.longitude = htobe32(uint32_t(remoteData.longitude));
            crsf_gps.groundspeed = htobe16(uint16_t(remoteData.groundSpeed * 3.6 * 10));
            int heading;
            if (remoteData.heading > 180) {
              heading = remoteData.heading - 360;
            } else {
              heading = remoteData.heading;
            }
            // NOTE: For the heading, I also had to edit the heading sensor on my transmitter and change the precision from "0.00" to "0.0"
            crsf_gps.heading = htobe16(uint16_t(heading * 100));  // NOTE: Heading comes from frame A
            crsf_gps.altitude = htobe16(uint16_t(remoteData.altitude / 100 + 1000));
            crsf_gps.satellites = uint8_t(remoteData.gpsSats);

            crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_GPS, &crsf_gps, sizeof(crsf_gps));  // TODO: Use CRSF_FRAME_GPS_PAYLOAD_SIZE?
        }

        if (frameType == 'X') {
            remoteData.hdop = readInt(0);
            remoteData.sensorStatus = readByte(2);

            /////////////////////////

            // LTM
            // HDOP	- uint16 HDOP * 100
            // hw status - uint8
            // LTM_X_counter - uint8
            // Disarm Reason - uint8
            // (unused) - 1 byte
        }
        
        state = IDLE;
        memset(serialBuffer, 0, LONGEST_FRAME_LENGTH);

      } else {
        /*
         * If no, put data into buffer
         */
        serialBuffer[receiverIndex++] = data;
      }

    }

  }
}

//////////////////////////////////////////////////////////////////




/***
 * This callback is called whenever new channel values are available.
 * Use crsf.getChannel(x) to get us channel values (1-16).
 ***/
static void packetChannels() {
  for(int i=0; i<CHANNEL_NUMBER; i++) {
    // NOTE:
    //    The CRSF channels are indexed from 1.
    //    The PPM channels are indexed from 0.
    ppm[i] = crsf.getChannel(i+1);
  }
}

static void crsfLinkUp() {
  digitalWrite(LED_BUILTIN, HIGH);
}

static void crsfLinkDown() {
  digitalWrite(LED_BUILTIN, LOW);

  for(int i=0; i<CHANNEL_NUMBER; i++) {
    ppm[i] = CHANNEL_FAILSAFE_VALUE;
  }
 }




void setup() {

  ////////////////////// PPM SETUP //////////////////////

  //initiallize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++) {
      ppm[i] = CHANNEL_DEFAULT_VALUE;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

  //////////////////////////////////////////////////////////////////



  ////////////////////// CRSF SETUP //////////////////////

  Serial.begin(115200);

  // If something other than changing the baud of the UART needs to be done, do it here
  // Serial1.end(); Serial1.begin(500000, SERIAL_8N1, 16, 17);

  // Attach the channels callback
  crsf.onPacketChannels = &packetChannels;
  crsf.onLinkUp = &crsfLinkUp;
  crsf.onLinkDown = &crsfLinkDown;

  //////////////////////////////////////////////////////////////////


  ////////////////////// LTM SETUP //////////////////////
  ltmSerial.begin(9600);
  //////////////////////////////////////////////////////////////////

}


void loop() {
  crsf.loop();
  checkLTM();
}



