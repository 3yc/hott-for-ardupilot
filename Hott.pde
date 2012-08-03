// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// HoTT v4 telemetry for ArduCopter
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//
// Check project homepage at https://code.google.com/p/hott-for-ardupilot/
//
// 08/2012 by Adam Majerczyk (majerczyk.adam@gmail.com)
// v0.9.1b (beta software)
//
// Developed and tested with:
// Transmitter MC-32 v1.030
// Receiver GR-16 v6a10_f9
// Receiver GR-12 v3a40_e9
//

#ifdef HOTT_TELEMETRY

#if HOTT_TELEMETRY_SERIAL_PORT == 2
	FastSerialPort2(Serial2);      //HOTT serial port
	#define _HOTT_PORT	Serial2
#else
#error HOTT serial port undefined. Please define HOTT_TELEMETRY_SERIAL_PORT
#endif


#define HOTT_TEXT_MODE_REQUEST_ID	0x7f
#define HOTT_BINARY_MODE_REQUEST_ID	0x80
//Sensor Ids
//Graupner #33611 General Air Module
#define HOTT_TELEMETRY_GAM_SENSOR_ID	0x8d
//Graupner #33600 GPS Module
#define HOTT_TELEMETRY_GPS_SENSOR_ID	0x8a
//Graupner #33620 Electric Air Module
#define HOTT_TELEMETRY_EAM_SENSOR_ID	0x8e
//Graupner #33601 Vario Module
#define HOTT_TELEMETRY_VARIO_SENSOR_ID	0x89

//Textmode key codes
#define HOTT_TEXT_MODE_ESC		0x07
#define HOTT_TEXT_MODE_INC		0x0D
#define HOTT_TEXT_MODE_DEC		0x0B
#define HOTT_TEXT_MODE_ENTER	        0x0E
//Inc+Dec -> Set button on transmittier
#define HOTT_TEXT_MODE_INC_DEC	        0x09

static boolean _hott_telemetry_is_sending = false;
static byte _hott_telemetry_sendig_msgs_id = 0;

#define HOTT_TEXTMODE_MSG_TEXT_LEN 168
//Text mode msgs type
struct HOTT_TEXTMODE_MSG {
	byte start_byte;	//#01 constant value 0x7b
	byte fill1;			//#02 constant value 0x00
	byte warning_beeps;	//#03 1=A 2=B ...
	byte msg_txt[HOTT_TEXTMODE_MSG_TEXT_LEN];	//#04 ASCII text to display to
							// Bit 7 = 1 -> Inverse character display
                            // Display 21x8
	byte stop_byte;		//#171 constant value 0x7d
	byte parity;		//#172 Checksum / parity
};
static HOTT_TEXTMODE_MSG hott_txt_msg;


#ifdef HOTT_SIM_GAM_SENSOR
struct HOTT_GAM_MSG {
	byte start_byte;			//#01 start byte constant value 0x7c
	byte gam_sensor_id; 		//#02 EAM sensort id. constat value 0x8d
	byte warning_beeps;			//#03 1=A 2=B ... 0x1a=Z  0 = no alarm
	byte sensor_id;	            //#04 constant value 0xd0
	byte alarm_invers1;			//#05 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm field
								// 0	all cell voltage
								// 1	Battery 1
								// 2	Battery 2
								// 3	Temperature 1
								// 4	Temperature 2
								// 5	Fuel
								// 6	mAh
								// 7	Altitude
	byte alarm_invers2;			//#06 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm Field
								// 0	main power current
								// 1	main power voltage
								// 2	Altitude
								// 3	m/s				
								// 4	m/3s
								// 5	unknown
								// 6	unknown
								// 7	"ON" sign/text msg active
								
	byte cell1;					//#07 cell 1 voltage lower value. 0.02V steps, 124=2.48V
	byte cell2;					//#08
	byte cell3;					//#09
	byte cell4;					//#10
	byte cell5;					//#11
	byte cell6;					//#12
	byte batt1_L;				//#13 battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V
	byte batt1_H;				//#14
	byte batt2_L;				//#15 battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V
	byte batt2_H;				//#16
	byte temperature1;			//#17 temperature 1. offset of 20. a value of 20 = 0°C
	byte temperature2;			//#18 temperature 2. offset of 20. a value of 20 = 0°C
	byte fuel_procent;			//#19 Fuel capacity in %. Values 0--100
								// graphical display ranges: 0-25% 50% 75% 100%
	byte fuel_ml_L;				//#20 Fuel in ml scale. Full = 65535!
	byte fuel_ml_H;				//#21
	byte rpm_L;					//#22 RPM in 10 RPM steps. 300 = 3000rpm
	byte rpm_H;					//#23
	byte altitude_L;			//#24 altitude in meters. offset of 500, 500 = 0m
	byte altitude_H;			//#25
	byte climbrate_L;			//#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
	byte climbrate_H;			//#27
	byte climbrate3s;			//#28 climb rate in m/3sec. Value of 120 = 0m/3sec
	byte current_L;				//#29 current in 0.1A steps
	byte current_H;				//#30
	byte main_voltage_L;		//#31 Main power voltage using 0.1V steps
	byte main_voltage_H;		//#32
	byte batt_cap_L;			//#33 used battery capacity in 10mAh steps
	byte batt_cap_H;			//#34
	byte speed_L;				//#35 (air?) speed in km/h(?) we are using ground speed here per default
	byte speed_H;				//#36
	byte min_cell_volt;			//#37 minimum cell voltage in 2mV steps. 124 = 2,48V
	byte min_cell_volt_num;		//#38 number of the cell with the lowest voltage
	byte rpm2_L;				//#39 RPM in 10 RPM steps. 300 = 3000rpm
	byte rpm2_H;				//#40
	byte general_error_number;	//#41 Voice error == 12. TODO: more docu
	byte pressure;				//#42 Pressure up to 16bar. 0,1bar scale. 20 = 2bar
	byte version;				//#43 version number TODO: more info?
	byte stop_byte;				//#44 stop byte
	byte parity;				//#45 CRC/Parity
};

static struct HOTT_GAM_MSG hott_gam_msg;
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
#define HOTT_VARIO_MSG_TEXT_LEN	21

struct HOTT_VARIO_MSG {
	byte start_byte;			//#01 start byte constant value 0x7c
	byte vario_sensor_id; 		//#02 VARIO sensort id. constat value 0x89
	byte warning_beeps;			//#03 1=A 2=B ...
	byte sensor_id;	            //#04 constant value 0x90
	byte inverse_status;		//#05 Inverse display (alarm?) bitmask
								//TODO: more info
	byte altitude_L;			//#06 Altitude low byte. In meters. A value of 500 means 0m
	byte altitude_H;			//#07 Altitude high byte
	byte altitude_max_L;		//#08 Max. measured altitude low byte. In meters. A value of 500 means 0m
	byte altitude_max_H;		//#09 Max. measured altitude high byte
	byte altitude_min_L;		//#10 Min. measured altitude low byte. In meters. A value of 500 means 0m
	byte altitude_min_H;		//#11 Min. measured altitude high byte
	byte climbrate_L;			//#12 Climb rate in m/s. Steps of 0.01m/s. Value of 30000 = 0.00 m/s
	byte climbrate_H;			//#13 Climb rate in m/s
	byte climbrate3s_L;			//#14 Climb rate in m/3s. Steps of 0.01m/3s. Value of 30000 = 0.00 m/3s
	byte climbrate3s_H;			//#15 Climb rate m/3s low byte
	byte climbrate10s_L;		//#16 Climb rate m/10s. Steps of 0.01m/10s. Value of 30000 = 0.00 m/10s
	byte climbrate10s_H;		//#17 Climb rate m/10s low byte
	byte text_msg[HOTT_VARIO_MSG_TEXT_LEN];			//#18 Free ASCII text message
	byte free_char1;			//#39 Free ASCII character.  appears right to home distance
	byte free_char2;			//#40 Free ASCII character.  appears right to home direction
	byte free_char3;			//#41 Free ASCII character.  appears? TODO: Check where this char appears
	byte compass_direction;		//#42 Compass heading in 2° steps. 1 = 2°
	byte version;				//#43 version number TODO: more info?
	byte stop_byte;				//#44 stop byte, constant value 0x7d
	byte parity;				//#45 checksum / parity
};

static HOTT_VARIO_MSG hott_vario_msg;
#endif

#ifdef HOTT_SIM_EAM_SENSOR
struct HOTT_EAM_MSG {
	byte start_byte;			//#01 start byte
	byte eam_sensor_id; 		//#02 EAM sensort id. constat value 0x8e
	byte warning_beeps;			//#03 1=A 2=B ... or 'A' - 0x40 = 1
								// Q	Min cell voltage sensor 1
								// R	Min Battery 1 voltage sensor 1
								// J	Max Battery 1 voltage sensor 1
								// F	Mim temperature sensor 1
								// H	Max temperature sensor 1
								// S	Min cell voltage sensor 2
								// K	Max cell voltage sensor 2
								// G	Min temperature sensor 2
								// I	Max temperature sensor 2
								// W	Max current
								// V	Max capacity mAh
								// P	Min main power voltage
								// X	Max main power voltage
								// O	Min altitude
								// Z	Max altitude
								// C	(negative) sink rate m/sec to high
								// B	(negative) sink rate m/3sec to high
								// N	climb rate m/sec to high
								// M	climb rate m/3sec to high
								
	byte sensor_id;	            //#04 constant value 0xe0
	byte alarm_invers1;			//#05 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm field
								// 0	mAh
								// 1	Battery 1
								// 2	Battery 2
								// 3	Temperature 1
								// 4	Temperature 2
								// 5	Altitude
								// 6	Current
								// 7	Main power voltage
	byte alarm_invers2;			//#06 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm Field
								// 0	m/s
								// 1	m/3s
								// 2	Altitude (duplicate?)
								// 3	m/s	(duplicate?)					
								// 4	m/3s (duplicate?)
								// 5	unknown/unused
								// 6	unknown/unused
								// 7	"ON" sign/text msg active
								
	byte cell1_L;				//#07 cell 1 voltage lower value. 0.02V steps, 124=2.48V
	byte cell2_L;				//#08
	byte cell3_L;				//#09
	byte cell4_L;				//#10
	byte cell5_L;				//#11
	byte cell6_L;				//#12
	byte cell7_L;				//#13
	byte cell1_H;				//#14 cell 1 voltage high value. 0.02V steps, 124=2.48V
	byte cell2_H;				//#15
	byte cell3_H;				//#16
	byte cell4_H;				//#17
	byte cell5_H;				//#18
	byte cell6_H;				//#19
	byte cell7_H;				//#20
	
	byte batt1_voltage_L;		//#21 battery 1 voltage lower value. opt. cell8_L 0.02V steps
	byte batt1_voltage_H;		//#22
	
	byte batt2_voltage_L;		//#23 battery 2 voltage lower value. opt cell8_H value-. 0.02V steps
	byte batt2_voltage_H;		//#24
	
	byte temp1;					//#25 Temperature sensor 1. 0°=20, 26°=46
	byte temp2;					//#26 temperature sensor 2
	
	byte altitude_L;			//#27 Attitude lower value. unit: meters. Value of 500 = 0m
	byte altitude_H;			//#28
	
	byte current_L;				//#29 Current in 0.1 steps
	byte current_H;				//#30
	
	byte main_voltage_L;		//#30 Main power voltage (drive) in 0.1V steps
	byte main_voltage_H;		//#31
	
	byte batt_cap_L;			//#32 used battery capacity in 10mAh steps
	byte batt_cap_H;			//#33
	
	byte climbrate_L;			//#34 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
	byte climbrate_H;			//#35
	
	byte climbrate3s;			//#36 climbrate in m/3sec. Value of 120 = 0m/3sec
	
	byte rpm_L;					//#37 RPM. Steps: 10 U/min
	byte rpm_H;					//#38
	
	byte electric_min;			//#39 Electric minutes. Time does start, when motor current is > 3 A
	byte electric_sec;			//#40
	
	byte speed_L;				//#41 (air?) speed in km/h. Steps 1km/h
	byte speed_H;				//#42
	byte stop_byte;				//#43 stop byte
	byte parity;				//#44 CRC/Parity
};

static HOTT_EAM_MSG hott_eam_msg;
#endif

#ifdef HOTT_SIM_GPS_SENSOR
//HoTT GPS Sensor response to Receiver (?!not?! Smartbox)
struct HOTT_GPS_MSG {
  byte start_byte;      //#01 constant value 0x7c
  byte gps_sensor_id;   //#02 constant value 0x8a
  byte warning_beeps;   //#03 1=A 2=B ...
  byte sensor_id;       //#04 constant (?) value 0xa0
  byte inverse_status;	//#05
  						//TODO: more info
  byte inverse_status_status;	//#06  1 = No GPS signal
  								//TODO: more info

  byte flight_direction; //#07 flight direction in 2 degreees/step (1 = 2degrees);
  byte gps_speed_L;  //08 km/h
  byte gps_speed_H;  //#09
  
  byte pos_NS;  //#10 north = 0, south = 1
  byte pos_NS_dm_L;  //#11 degree minutes ie N48°39’988
  byte pos_NS_dm_H;  //#12
  byte pos_NS_sec_L; //#13 position seconds
  byte pos_NS_sec_H;  //#14

  byte pos_EW;  //#15 east = 0, west = 1
  byte pos_EW_dm_L; //#16 degree minutes ie. E9°25’9360
  byte pos_EW_dm_H;  //#17
  byte pos_EW_sec_L;  //#18 position seconds
  byte pos_EW_sec_H;  //#19
  
  byte home_distance_L;  //#20 meters
  byte home_distance_H;  //#21
  
  byte altitude_L; //#22 meters. Value of 500 = 0m 
  byte altitude_H; //#23
  
  byte climbrate_L;  //#24 m/s 0.01m/s resolution. Value of 30000 = 0.00 m/s
  byte climbrate_H;  //#25
  
  byte climbrate3s;  //#26 climbrate in m/3s resolution, value of 120 = 0 m/3s
  
  byte gps_satelites;  //#27 sat count
  byte gps_fix_char; //#28 GPS fix character. display, 'D' = DGPS, '2' = 2D, '3' = 3D, '-' = no fix. Where appears this char???
  
  byte home_direction; //#29 direction from starting point to Model position (2 degree steps)
  byte angle_roll;    //#30 angle roll in 2 degree steps
  byte angle_nick;    //#31 angle in 2degree steps
  byte angle_compass; //#32 angle in 2degree steps. 1 = 2°, 255 = - 2° (1 byte) North = 0°
  
  byte gps_time_h;  //#33 UTC time hours
  byte gps_time_m;  //#34 UTC time minutes
  byte gps_time_s;  //#35 UTC time seconds
  byte gps_time_sss;//#36 UTC time milliseconds
  
  byte msl_altitude_L;  //#37 mean sea level altitude
  byte msl_altitude_H;  //#38
  
  byte vibration; //#39 vibrations level in %
  
  byte free_char1;  //#40 appears right to home distance
  byte free_char2;  //#41 appears right to home direction
  byte free_char3;  //#42 GPS ASCII D=DGPS 2=2D 3=3D -=No Fix
  byte version;  //#43
  				 // 0	GPS Graupner #33600
                 // 1	Gyro Receiver
                 // 255 Mikrokopter
  byte end_byte;  //#44 constant value 0x7d
  byte parity;    //#45
};

static HOTT_GPS_MSG hott_gps_msg;
#endif

// HoTT serial send buffer pointer
static byte *_hott_msg_ptr = 0;
// Len of HoTT serial buffer
static int _hott_msg_len = 0;

#if HOTT_TELEMETRY_SERIAL_PORT == 2
void _hott_enable_transmitter() {
  //enables serial transmitter, disables receiver
    UCSR2B &= ~_BV(RXEN2);
    UCSR2B |= _BV(TXEN2); 
}

void _hott_enable_receiver() {
   //enables serial receiver, disables transmitter
  UCSR2B &= ~_BV(TXEN2);
  UCSR2B |= _BV(RXEN2);
}
#else
	//TODO: add more serial port definitions
	#error please define your port settings...
#endif

//*****************************************************************************

/*
  Initializes a HoTT GPS message (Receiver answer type  !Not Smartbox)
*/
void _hott_msgs_init() {
#ifdef HOTT_SIM_GPS_SENSOR
  memset(&hott_gps_msg, 0, sizeof(struct HOTT_GPS_MSG));
  hott_gps_msg.start_byte = 0x7c;
  hott_gps_msg.gps_sensor_id = 0x8a;
  hott_gps_msg.sensor_id = 0xa0;
  
  hott_gps_msg.version = 0x00;
  hott_gps_msg.end_byte = 0x7d;
//  hott_gps_msg.parity = _hott_checksum((byte *)&hott_gps_msg, sizeof(struct HOTT_GPS_MSG));
#endif

#ifdef HOTT_SIM_EAM_SENSOR
 memset(&hott_eam_msg, 0, sizeof(struct HOTT_EAM_MSG));
 hott_eam_msg.start_byte = 0x7c;
 hott_eam_msg.eam_sensor_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
 hott_eam_msg.sensor_id = 0xe0;
 hott_eam_msg.stop_byte = 0x7d;
// hott_eam_msg.parity = _hott_checksum((byte *)&hott_eam_msg, sizeof(struct HOTT_EAM_MSG));
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
 memset(&hott_vario_msg, 0, sizeof(struct HOTT_VARIO_MSG));
 hott_vario_msg.start_byte = 0x7c;
 hott_vario_msg.vario_sensor_id = HOTT_TELEMETRY_VARIO_SENSOR_ID;
 hott_vario_msg.sensor_id = 0x90;
 hott_vario_msg.stop_byte = 0x7d;
#endif

#ifdef HOTT_SIM_GAM_SENSOR
 memset(&hott_gam_msg, 0, sizeof(struct HOTT_GAM_MSG));
 hott_gam_msg.start_byte = 0x7c;
 hott_gam_msg.gam_sensor_id = HOTT_TELEMETRY_GAM_SENSOR_ID;
 hott_gam_msg.sensor_id = 0xd0;
 hott_gam_msg.stop_byte = 0x7d;
#endif
 
  memset(&hott_txt_msg, 0, sizeof(struct HOTT_TEXTMODE_MSG));
  hott_txt_msg.start_byte = 0x7b;
  hott_txt_msg.fill1 = 0x00;
  hott_txt_msg.warning_beeps = 0x00;
  hott_txt_msg.stop_byte = 0x7d;

  sprintf((char *)hott_txt_msg.msg_txt,"%s",THISFIRMWARE);
  sprintf((char *)&hott_txt_msg.msg_txt[1*21],"by Adam Majerczyk");
  sprintf((char *)&hott_txt_msg.msg_txt[2*21],"adam@3yc.de");
  sprintf((char *)&hott_txt_msg.msg_txt[4*21],"more to come!");

  hott_txt_msg.parity = _hott_checksum((byte *)&hott_txt_msg, sizeof(struct HOTT_TEXTMODE_MSG));
}

/*
  called by timer_scheduler
*/
static uint32_t _hott_serial_timer;
/*
  Called periodically (≈1ms) by timer scheduler
  
  @param tnow  timestamp in uSecond
*/
void _hott_serial_scheduler(uint32_t tnow) {
   _hott_check_serial_data(tnow);	// Check for data request
  if(_hott_msg_ptr == 0) return;   //no data to send
  if(_hott_telemetry_is_sending) {
    //we are sending already, wait for a delay of 2ms between data bytes
    if(tnow - _hott_serial_timer < 3000)  //delay ca. 3,5 mS. 19200 baud = 520uS / Byte + 3ms required delay
    										// Graupner Specs claims here 2ms but in reality it's 3ms, they using 3ms too...
      return;
  } else {
  	//new data request
  	tnow = micros(); //correct the 5ms  delay in _hott_check_serial_data()...
  }
  _hott_send_telemetry_data();
  _hott_serial_timer = tnow;
}

/*
  Transmitts a HoTT message
*/
void _hott_send_telemetry_data() {

  if(!_hott_telemetry_is_sending) {
  	// new message to send
    _hott_telemetry_is_sending = true;
    _hott_enable_transmitter();  //switch to transmit mode  
  }

  //send data
  if(_hott_msg_len == 0) {
    //all data send
    _hott_msg_ptr = 0;
    _hott_telemetry_is_sending = false;
    _hott_enable_receiver();
    _HOTT_PORT.flush();
  } else {
    --_hott_msg_len;
    _HOTT_PORT.write(*_hott_msg_ptr++);
  }
}

/*
  Onetime setup for HoTT
*/
void _hott_setup() {
  _HOTT_PORT.begin(19200);
  _hott_enable_receiver();
  timer_scheduler.register_process(_hott_serial_scheduler);

  //init msgs
  _hott_msgs_init();  //set default values  
}

/*
  Calculates HoTT message checksum/parity
*/
byte _hott_checksum(byte *buffer, int len) {
  int checksum = 0;
  for(int i=0; i< len -1; i++) {
    checksum += buffer[i];
  }
  return (byte) checksum;
}


/*
  Check for a valid HoTT requests on serial bus
*/
static uint32_t _hott_serial_request_timer = 0;

void _hott_check_serial_data(uint32_t tnow) {
	if(_hott_telemetry_is_sending == true) return;
    if(_HOTT_PORT.available() > 1) {
      if(_HOTT_PORT.available() == 2) {
        if(_hott_serial_request_timer == 0) {
        	//new request, check required
        	_hott_serial_request_timer = tnow;	//save timestamp
        	return;
        } else {
        	if(tnow - _hott_serial_request_timer < 4600)	//wait ca. 5ms
        		return;
        	_hott_serial_request_timer = 0;	//clean timer
        }
        // we never reach this point if there is additionally data on bus, so is has to be valid request
        unsigned char c = _HOTT_PORT.read();
        unsigned char addr = _HOTT_PORT.read();
        //ok, we have a valid request, check for address
        switch(c) {
//*****************************************************************************
          case HOTT_TEXT_MODE_REQUEST_ID:
           //Text mode
             {
             byte tmp = (addr >> 4);
#ifdef HOTT_SIM_GPS_SENSOR
				//GPS module text mode
             if(tmp == (HOTT_TELEMETRY_GPS_SENSOR_ID & 0x0f)) {
               memset((char *)&hott_txt_msg.msg_txt[3*21], 0x20, 21); //clear line
               sprintf((char *)&hott_txt_msg.msg_txt[3*21],_hott_invert_all_chars("GPS sensor module"));
               hott_txt_msg.parity = _hott_checksum((byte *)&hott_txt_msg, sizeof(struct HOTT_TEXTMODE_MSG));
               _hott_send_text_msg();	//send message
             }
#endif
#ifdef HOTT_SIM_EAM_SENSOR
             if(tmp == (HOTT_TELEMETRY_EAM_SENSOR_ID & 0x0f)) {
               memset((char *)&hott_txt_msg.msg_txt[3*21], 0x20, 21); //clear line
               sprintf((char *)&hott_txt_msg.msg_txt[3*21],_hott_invert_all_chars("EAM sensor module"));
               hott_txt_msg.parity = _hott_checksum((byte *)&hott_txt_msg, sizeof(struct HOTT_TEXTMODE_MSG));
               _hott_send_text_msg();	//send message
             }
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
             if(tmp == (HOTT_TELEMETRY_VARIO_SENSOR_ID & 0x0f)) {
               memset((char *)&hott_txt_msg.msg_txt[3*21], 0x20, 21); //clear line
               sprintf((char *)&hott_txt_msg.msg_txt[3*21],_hott_invert_all_chars("VARIO sensor module"));
               hott_txt_msg.parity = _hott_checksum((byte *)&hott_txt_msg, sizeof(struct HOTT_TEXTMODE_MSG));
               _hott_send_text_msg();	//send message
             }
#endif
#ifdef HOTT_SIM_GAM_SENSOR
             if(tmp == (HOTT_TELEMETRY_GAM_SENSOR_ID & 0x0f)) {
               memset((char *)&hott_txt_msg.msg_txt[3*21], 0x20, 21); //clear line
               sprintf((char *)&hott_txt_msg.msg_txt[3*21],_hott_invert_all_chars("GAM sensor module"));
               hott_txt_msg.parity = _hott_checksum((byte *)&hott_txt_msg, sizeof(struct HOTT_TEXTMODE_MSG));
               _hott_send_text_msg();	//send message
             }
#endif

           }
           break;
//*****************************************************************************
          case HOTT_BINARY_MODE_REQUEST_ID:
#ifdef HOTT_SIM_GPS_SENSOR
			//GPS module binary mode
            if(addr == HOTT_TELEMETRY_GPS_SENSOR_ID) {
//            Serial.printf("\n%ld: REQ_GPS",micros() / 1000);
              _hott_send_gps_msg();
              break;
            }
#endif
#ifdef HOTT_SIM_EAM_SENSOR
            if(addr == HOTT_TELEMETRY_EAM_SENSOR_ID) {
//            Serial.printf("\n%ld: REQ_EAM",micros() / 1000);
		      _hott_send_eam_msg();
              break;
		    }
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
            if(addr == HOTT_TELEMETRY_VARIO_SENSOR_ID) {
//            Serial.printf("\n%ld: REQ_VARIO",micros() / 1000);
		      _hott_send_vario_msg();
              break;
		    }
#endif
#ifdef HOTT_SIM_GAM_SENSOR
            if(addr == HOTT_TELEMETRY_GAM_SENSOR_ID) {
//            Serial.printf("\n%ld: REQ_GAM",micros() / 1000);
		      _hott_send_gam_msg();
              break;
		    }
#endif

//END: case HOTT_BINARY_MODE_REQUEST_ID:
           break;
//*****************************************************************************         
          default:
            //Serial.printf("0x%02x Mode for 0x%02x\n", c, addr);
            break;
        }
      } else {
        //ignore data from other sensors
        _HOTT_PORT.flush();
        _hott_serial_request_timer = 0;
      }
    }
}

void _hott_send_msg(byte *buffer, int len) {
  if(_hott_telemetry_is_sending == true) return;
  _hott_msg_ptr = buffer;
  _hott_msg_len = len;
}

#ifdef HOTT_SIM_GAM_SENSOR
void _hott_send_gam_msg() {
	_hott_send_msg((byte *)&hott_gam_msg, sizeof(struct HOTT_GAM_MSG));
  _hott_telemetry_sendig_msgs_id = HOTT_TELEMETRY_GAM_SENSOR_ID;
}
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
void _hott_send_vario_msg() {
	_hott_send_msg((byte *)&hott_vario_msg, sizeof(struct HOTT_VARIO_MSG));
	_hott_telemetry_sendig_msgs_id = HOTT_TELEMETRY_VARIO_SENSOR_ID;
}
#endif

#ifdef HOTT_SIM_GPS_SENSOR
void _hott_send_gps_msg() {
  _hott_send_msg((byte *)&hott_gps_msg, sizeof(struct HOTT_GPS_MSG));
  _hott_telemetry_sendig_msgs_id = HOTT_TELEMETRY_GPS_SENSOR_ID;
}
#endif

#ifdef HOTT_SIM_EAM_SENSOR
//Send EMA sensor data
void _hott_send_eam_msg() {
  _hott_send_msg((byte *)&hott_eam_msg, sizeof(struct HOTT_EAM_MSG));
  _hott_telemetry_sendig_msgs_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
}
#endif

void _hott_send_text_msg() {
  _hott_send_msg((byte *)&hott_txt_msg, sizeof(struct HOTT_TEXTMODE_MSG));
  _hott_telemetry_sendig_msgs_id = HOTT_TEXT_MODE_REQUEST_ID;
}

#ifdef HOTT_SIM_GAM_SENSOR
void _hott_update_gam_msg() {
	hott_gam_msg.temperature1 = (byte)((barometer.get_temperature() / 10) + 20);
	hott_gam_msg.temperature2 = 20; // 0°C
	(int &)hott_gam_msg.altitude_L = (int)((current_loc.alt - home.alt) / 100)+500;
	(int &)hott_gam_msg.climbrate_L = 30000 + climb_rate_actual;
	hott_gam_msg.climbrate3s = 120 + (climb_rate / 100);  // 0 m/3s using filtered data here
	(int &)hott_gam_msg.current_L = current_amps1*10;
	(int &)hott_gam_msg.main_voltage_L = (int)(battery_voltage1 * 10.0);
	(int &)hott_gam_msg.batt_cap_L = current_total1 / 10;
	hott_gam_msg.fuel_procent = (100.0 * (g.pack_capacity - current_total1) / g.pack_capacity);	// my fuel are electrons :)
	(int &)hott_gam_msg.speed_L = (int)((float)(g_gps->ground_speed * 0.036));

    //display ON when motors are armed
    if (motors.armed()) {
    	hott_gam_msg.alarm_invers2 |= 0x80;
    } else {
        hott_gam_msg.alarm_invers2 &= 0x7f;
    }

	//calc checksum
	hott_gam_msg.parity = _hott_checksum((byte *)&hott_gam_msg, sizeof(struct HOTT_GAM_MSG));
}
#endif

#ifdef HOTT_SIM_EAM_SENSOR
//Update EAM sensor data
void _hott_update_eam_msg() {
	(int &)hott_eam_msg.batt1_voltage_L = (int)(0);
	(int &)hott_eam_msg.batt2_voltage_L = (int)(0);
	hott_eam_msg.temp1 = (byte)((barometer.get_temperature() / 10) + 20);
	hott_eam_msg.temp2 = 20;	//0°
	(int &)hott_eam_msg.altitude_L = (int)((current_loc.alt - home.alt) / 100)+500;
	(int &)hott_eam_msg.current_L = current_amps1*10;
	(int &)hott_eam_msg.main_voltage_L = (int)(battery_voltage1 * 10.0);
	(int &)hott_eam_msg.batt_cap_L = current_total1 / 10;
	(int &)hott_eam_msg.speed_L = (int)((float)(g_gps->ground_speed * 0.036));

  	(int &)hott_eam_msg.climbrate_L = 30000 + climb_rate_actual;  
  	hott_eam_msg.climbrate3s = 120 + (climb_rate / 100);  // 0 m/3s using filtered data here
  	
        //display ON when motors are armed
        if (motors.armed()) {
          hott_eam_msg.alarm_invers2 |= 0x80;
        } else {
          hott_eam_msg.alarm_invers2 &= 0x7f;
        }
    //hott_eam_msg.warning_beeps = 'V' - 0x40;
	hott_eam_msg.parity = _hott_checksum((byte *)&hott_eam_msg, sizeof(struct HOTT_EAM_MSG));
}
#endif

#ifdef HOTT_SIM_GPS_SENSOR
// Updates GPS message values
void _hott_update_gps_msg() { 
  // update GPS telemetry data
  (int &)hott_gps_msg.msl_altitude_L = (int)g_gps->altitude / 100;  //meters above sea level  
  hott_gps_msg.flight_direction = (byte)(g_gps->ground_course / 200);  // in 2* steps
  (int &)hott_gps_msg.gps_speed_L = (int)((float)(g_gps->ground_speed * 0.036));

  
  if(g_gps->status() == GPS::GPS_OK) {
    hott_gps_msg.inverse_status_status = 0;
    hott_gps_msg.gps_fix_char = '3';  
    hott_gps_msg.free_char3 = '3';  //3D Fix according to specs...
  } else {
    //No GPS Fix
    hott_gps_msg.inverse_status_status = 1;
    hott_gps_msg.gps_fix_char = '-';
    hott_gps_msg.free_char3 = '-';
    (int &)hott_gps_msg.home_distance_L = (int)0; // set distance to 0 since there is no GPS signal
  }
  
  switch(control_mode) {
	case AUTO:
        case LOITER:
          //Use home direction field to display direction an distance to next waypoint
          {
          	  int32_t dist = get_distance_cm(&current_loc, &next_WP);
	          (int &)hott_gps_msg.home_distance_L = dist < 0 ? 0 : dist / 100;
    	      hott_gps_msg.home_direction = get_bearing(&current_loc, &next_WP) / 200; //get_bearing() return value in degrees * 100
        	  //Display WP to mark the change of meaning!
           	hott_gps_msg.free_char1 ='W';
           	hott_gps_msg.free_char2 ='P';
           }
           break;

        default:
        //Display Home direction and distance
        {
          int32_t dist = get_distance_cm(&current_loc, &home);
          (int &)hott_gps_msg.home_distance_L = dist < 0 ? 0 : dist / 100;
          hott_gps_msg.home_direction = get_bearing(&current_loc, &home) / 200; //get_bearing() return value in degrees * 100
          hott_gps_msg.free_char1 = 0;
          hott_gps_msg.free_char2 = 0;
          break;
        }
	}

  
  //
  //latitude
  float coor = current_loc.lat / 10000000.0;
  if(coor < 0.0)
    coor *= -1.0;
  int lat = coor;  //degree
  coor -= lat;
  lat *= 100;
  
  coor *= 60;
  int tmp = coor;  //minutes
  lat += tmp;
  //seconds
  coor -= tmp;
  coor *= 10000.0;
  int lat_sec = coor;
  //
  // Longitude
  coor = current_loc.lng / 10000000.0;
  if(coor < 0.0)
    coor *= -1.0;
  int lng = coor;
  coor -= lng;
  lng *= 100;
  
  coor *= 60;
  tmp = coor;  //minutes
  lng += tmp;
  //seconds
  coor -= tmp;
  coor *= 10000.0;
  int lng_sec = coor;
  
  if(current_loc.lat >= 0) {
    hott_gps_msg.pos_NS = 0;  //north
  } else {
    hott_gps_msg.pos_NS = 1;  //south
  }
  (int &)hott_gps_msg.pos_NS_dm_L = (int)lat;
  (int &)hott_gps_msg.pos_NS_sec_L = (int)(lat_sec);
  
  if(current_loc.lng >= 0) {
     hott_gps_msg.pos_EW = 0; //east
  } else {
     hott_gps_msg.pos_EW = 1; //west
  }
  (int &)hott_gps_msg.pos_EW_dm_L = (int)(lng);
  (int &)hott_gps_msg.pos_EW_sec_L = (int)(lng_sec);
  
  (int &)hott_gps_msg.altitude_L = (int)((current_loc.alt - home.alt) / 100)+500;  //meters above ground

  (int &)hott_gps_msg.climbrate_L = 30000 + climb_rate_actual;  
  hott_gps_msg.climbrate3s = 120 + (climb_rate / 100);  // 0 m/3s
  
  hott_gps_msg.gps_satelites = (byte)g_gps->num_sats;
  
  hott_gps_msg.angle_roll = ahrs.roll_sensor / 200;
  hott_gps_msg.angle_nick = ahrs.pitch_sensor / 200;
  
  hott_gps_msg.angle_compass = ToDeg(compass.calculate_heading(ahrs.get_dcm_matrix())) / 2;
  //hott_gps_msg.flight_direction = hott_gps_msg.angle_compass;	//

  uint32_t t = g_gps->time;
  hott_gps_msg.gps_time_h = t / 3600000;
  t -= (hott_gps_msg.gps_time_h * 3600000);
  
  hott_gps_msg.gps_time_m = t / 60000;
  t -= hott_gps_msg.gps_time_m * 60000;
  
  hott_gps_msg.gps_time_s = t / 1000;
  hott_gps_msg.gps_time_sss = t - (hott_gps_msg.gps_time_s * 1000);
  
  // Calc checksum
  hott_gps_msg.parity = _hott_checksum((byte *)&hott_gps_msg, sizeof(struct HOTT_GPS_MSG));
}
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
static int _hott_max_altitude = 0;
static int _hott_min_altitude = 0;

void _hott_update_vario_msg() {
	(int &)hott_vario_msg.altitude_L = (int)((current_loc.alt - home.alt) / 100)+500;

	if( (current_loc.alt - home.alt) > _hott_max_altitude)
		_hott_max_altitude = (current_loc.alt - home.alt);
	(int &)hott_vario_msg.altitude_max_L = (int)(_hott_max_altitude / 100)+500;

	if( (current_loc.alt - home.alt) < _hott_min_altitude)
		_hott_min_altitude = (current_loc.alt - home.alt);
	(int &)hott_vario_msg.altitude_min_L = (int)(_hott_min_altitude / 100)+500;
	
	(int &)hott_vario_msg.climbrate_L = 30000 + climb_rate_actual;
	(int &)hott_vario_msg.climbrate3s_L = 30000 + climb_rate;	//using filtered data here
	(int &)hott_vario_msg.climbrate10s_L = 30000;	//TODO: calc this stuff
	
	hott_vario_msg.compass_direction = ToDeg(compass.calculate_heading(ahrs.get_dcm_matrix())) / 2;
	
	//Free text processing
	char mode[10];
	char armed[10];
	strcpy(mode,flight_mode_strings[control_mode]);
	
	if (motors.armed()) {
          strcpy(armed,"ARMED");
	} else {
          strcpy(armed,"DISAR");
	}
	memset(hott_vario_msg.text_msg,0x20,HOTT_VARIO_MSG_TEXT_LEN);
	snprintf((char*)hott_vario_msg.text_msg,HOTT_VARIO_MSG_TEXT_LEN, "%s %s", &armed, &mode);
	hott_vario_msg.parity = _hott_checksum((byte *)&hott_vario_msg, sizeof(struct HOTT_VARIO_MSG));
}
#endif

/*
  Converts a "black on white" string into inverted one "white on black"
  Works in text mode only!
*/
char * _hott_invert_chars(char *str, int cnt) {
	if(str == 0) return str;
	int len = strlen(str);
	if((len < cnt)  && cnt > 0) len = cnt;
	for(int i=0; i< len; i++) {
		str[i] = (byte)(0x80 + (byte)str[i]);
	}
	return str;
}

char * _hott_invert_all_chars(char *str) {
    return _hott_invert_chars(str, 0);
}


/*
  Updates HoTT Messages values. Called in slow loop
*/
void _hott_update_telemetry_data() {
  //no update while sending
  
#ifdef HOTT_SIM_GPS_SENSOR
  if(!(_hott_telemetry_is_sending && _hott_telemetry_sendig_msgs_id == HOTT_TELEMETRY_GPS_SENSOR_ID))
	_hott_update_gps_msg();
#endif
#ifdef HOTT_SIM_EAM_SENSOR
  if(!(_hott_telemetry_is_sending && _hott_telemetry_sendig_msgs_id == HOTT_TELEMETRY_EAM_SENSOR_ID))
	_hott_update_eam_msg();
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
  if(!(_hott_telemetry_is_sending && _hott_telemetry_sendig_msgs_id == HOTT_TELEMETRY_VARIO_SENSOR_ID))
	_hott_update_vario_msg();
#endif
#ifdef HOTT_SIM_GAM_SENSOR
  if(!(_hott_telemetry_is_sending && _hott_telemetry_sendig_msgs_id == HOTT_TELEMETRY_GAM_SENSOR_ID))
	_hott_update_gam_msg();
#endif
}

#endif	//End of #ifdef HOTT_TELEMETRY
