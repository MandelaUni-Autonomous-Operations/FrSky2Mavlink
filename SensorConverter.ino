/*
  Adapted from FrSky S-Port Telemetry Decoder library example
  by Pawel Spychalski
  
  Note that you need Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) or ATmega328P based (e.g. Pro Mini, Nano, Uno), the FrSkySportDecoder library and the MAVLink C library for this script to work
*/

// Uncomment the #define below to enable internal polling of data.
// Use only when there is no device in the S.Port chain (e.g. S.Port capable FrSky receiver) that normally polls the data.
#define POLLING_ENABLED

#include "FrSkySportSensor.h"
#include "FrSkySportSensorAss.h"
#include "FrSkySportSensorEsc.h"
#include "FrSkySportSensorFcs.h"
#include "FrSkySportSensorFlvss.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorRpm.h"
#include "FrSkySportSensorSp2uart.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportDecoder.h"
#include "SoftwareSerial.h"
#include "ardupilotmega/mavlink.h"                      //Include MAVLink C Library here, 

#define bRate 115200

// Instantiate Various sensors with their associated ID's
FrSkySportSensorFcs fcs(FrSkySportSensor::ID7);                                 // Create FCS-40A sensor with default ID (use ID8 for FCS-150A)
FrSkySportSensorFlvss flvss1(FrSkySportSensor::ID2);                            // Create FLVSS sensor with default ID             
FrSkySportSensorRpm rpm1(FrSkySportSensor::ID6);                                // Create RPM sensor with default ID
FrSkySportSensorRpm rpm2(FrSkySportSensor::ID9); 

mavlink_system_t mavlink_system = { 1, MAV_COMP_ID_PERIPHERAL };
                  
#ifdef POLLING_ENABLED
  #include "FrSkySportPollingSimple.h"
  FrSkySportDecoder decoder(new FrSkySportPollingSimple()); // Create telemetry object with simple polling
#else
  FrSkySportDecoder decoder;                           // Create decoder object without polling
#endif

// Instantiate message variable and preload with impossible values
uint16_t decodeResult;
uint16_t volts = -10;
uint16_t current = -10;
float rpm1_val = -1;
float rpm2_val = -1;



void setup()
{
  cli();                                          //stop interrupts
  
  //set timer1 interrupt at 10Hz to publish MAVLink messages on the timer overflow
  
  TCCR1A = 0;                                     // set entire TCCR1A register to 0
  TCCR1B = 0;                                     // same for TCCR1B
  TCNT1  = 0;                                     //initialize counter value to 0
  
  // set compare match register for 1hz increments
  
  OCR1A = 1563;                                   // = (16*10^6) / (10*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();                                          //allow interrupts

  // Configure the decoder serial port and sensors (remember to use & to specify a pointer to sensor)
  
  decoder.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_2, &fcs, &flvss1, &rpm1, &rpm2);
  Serial.begin(bRate);
}

void command_heartbeat() {    // Sends a MAVLink heartbeat message
  int sysid = 1;
  int compid = MAV_COMP_ID_PERIPHERAL;
  uint8_t system_type = MAV_TYPE_GCS;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = 0;
  uint32_t custom_mode = 0;
  uint8_t system_state = 0;

  //Initialize required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack heartbeat message
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to send to buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message
  Serial.write(buf, len);
}

void command_voltage(uint16_t lcell, int16_t cur){    //Sends a MAVLink battery message based on readings from the FLVSS sensor
    int sysid = 1;    //MAVLink System ID
    int compid = 158;   //MAVLink Component ID
  
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_battery2_pack(sysid, compid, &msg, lcell, cur);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    Serial.write(buf, len);

}

void command_rpm(float rpm1, float rpm2){     //Sends a MAVLink rpm message based on readings from the rpm sensor
    int sysid = 1;    //MAVLink System ID
    int compid = 158;   //MAVLink Component ID
  

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_rpm_pack(sysid, compid, &msg, rpm1, rpm2);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    Serial.write(buf, len);

}
ISR(TIMER1_COMPA_vect){       //timer1 interrupt 10Hz
  command_voltage((volts*10), current);     // Sends S.Port sensor data to FC via MAVLink over hardware serial connection
  command_rpm(rpm1_val, rpm2_val);
  command_heartbeat();
}


void loop()
{
  
  decodeResult = decoder.decode();    // Decodes the data from the FrSky S.Port Sensors

  // Update variables whenever new sensor data is available
  volts = flvss1.getCell1();
  current = fcs.getCurrent();
  rpm1_val = rpm1.getRpm(); //
  rpm2_val = rpm2.getRpm();


}
