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
// 01/2013 by Adam Majerczyk (majerczyk.adam@gmail.com)
// Texmode add-on by Michi (mamaretti32@gmail.com)
// v0.9.7.1b (beta software)
//
// Developed and tested with:
// Transmitter MC-32 (v1.030,v1.041), MX-20, MC-20
// Receiver GR-16 v6a10_f9
// Receiver GR-12 v3a40_e9
//

#ifdef HOTT_TELEMETRY

#if HOTT_TELEMETRY_SERIAL_PORT == 2
	//FastSerialPort2(Serial2);      //HOTT serial port
	#define _HOTT_PORT	hal.uartC
#endif
#if HOTT_TELEMETRY_SERIAL_PORT == 3
	//FastSerialPort3(Serial3);      //HOTT serial port
	#error Not supported yet
//	#define _HOTT_PORT	hal.uartC
#endif

#ifndef HOTT_TELEMETRY_SERIAL_PORT
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



static bool _hott_telemetry_is_sending = false;
static int8_t _hott_telemetry_sendig_msgs_id = 0;

#ifdef HOTT_SIM_TEXTMODE
#define HOTT_TEXTMODE_MSG_TEXT_LEN 168
//Text mode msgs type
struct HOTT_TEXTMODE_MSG {
	int8_t start_byte;	//#01 constant value 0x7b
	int8_t fill1;			//#02 constant value 0x00
	int8_t warning_beeps;	//#03 1=A 2=B ...
	int8_t msg_txt[HOTT_TEXTMODE_MSG_TEXT_LEN];	//#04 ASCII text to display to
							// Bit 7 = 1 -> Inverse character display
                            // Display 21x8
	int8_t stop_byte;		//#171 constant value 0x7d
//	int8_t parity;		//#172 Checksum / parity
};
static HOTT_TEXTMODE_MSG hott_txt_msg;
#endif

#ifdef HOTT_SIM_GAM_SENSOR
struct HOTT_GAM_MSG {
	int8_t start_byte;			//#01 start int8_t constant value 0x7c
	int8_t gam_sensor_id; 		//#02 EAM sensort id. constat value 0x8d
	int8_t warning_beeps;			//#03 1=A 2=B ... 0x1a=Z  0 = no alarm
								// Q	Min cell voltage sensor 1
								// R	Min Battery 1 voltage sensor 1
								// J	Max Battery 1 voltage sensor 1
								// F	Min temperature sensor 1
								// H	Max temperature sensor 1
								// S	Min Battery 2 voltage sensor 2
								// K	Max Battery 2 voltage sensor 2
								// G	Min temperature sensor 2
								// I	Max temperature sensor 2
								// W	Max current
								// V	Max capacity mAh
								// P	Min main power voltage
								// X	Max main power voltage
								// O	Min altitude
								// Z	Max altitude
								// C	negative difference m/s too high
								// A	negative difference m/3s too high
								// N	positive difference m/s too high
								// L	positive difference m/3s too high
								// T	Minimum RPM
								// Y	Maximum RPM

	int8_t sensor_id;	            //#04 constant value 0xd0
	int8_t alarm_invers1;			//#05 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm field
								// 0	all cell voltage
								// 1	Battery 1
								// 2	Battery 2
								// 3	Temperature 1
								// 4	Temperature 2
								// 5	Fuel
								// 6	mAh
								// 7	Altitude
	int8_t alarm_invers2;			//#06 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm Field
								// 0	main power current
								// 1	main power voltage
								// 2	Altitude
								// 3	m/s				
								// 4	m/3s
								// 5	unknown
								// 6	unknown
								// 7	"ON" sign/text msg active
								
	int8_t cell1;					//#07 cell 1 voltage lower value. 0.02V steps, 124=2.48V
	int8_t cell2;					//#08
	int8_t cell3;					//#09
	int8_t cell4;					//#10
	int8_t cell5;					//#11
	int8_t cell6;					//#12
	int8_t batt1_L;				//#13 battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V
	int8_t batt1_H;				//#14
	int8_t batt2_L;				//#15 battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V
	int8_t batt2_H;				//#16
	int8_t temperature1;			//#17 temperature 1. offset of 20. a value of 20 = 0°C
	int8_t temperature2;			//#18 temperature 2. offset of 20. a value of 20 = 0°C
	int8_t fuel_procent;			//#19 Fuel capacity in %. Values 0--100
								// graphical display ranges: 0-25% 50% 75% 100%
	int8_t fuel_ml_L;				//#20 Fuel in ml scale. Full = 65535!
	int8_t fuel_ml_H;				//#21
	int8_t rpm_L;					//#22 RPM in 10 RPM steps. 300 = 3000rpm
	int8_t rpm_H;					//#23
	int8_t altitude_L;			//#24 altitude in meters. offset of 500, 500 = 0m
	int8_t altitude_H;			//#25
	int8_t climbrate_L;			//#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
	int8_t climbrate_H;			//#27
	int8_t climbrate3s;			//#28 climb rate in m/3sec. Value of 120 = 0m/3sec
	int8_t current_L;				//#29 current in 0.1A steps
	int8_t current_H;				//#30
	int8_t main_voltage_L;		//#31 Main power voltage using 0.1V steps
	int8_t main_voltage_H;		//#32
	int8_t batt_cap_L;			//#33 used battery capacity in 10mAh steps
	int8_t batt_cap_H;			//#34
	int8_t speed_L;				//#35 (air?) speed in km/h(?) we are using ground speed here per default
	int8_t speed_H;				//#36
	int8_t min_cell_volt;			//#37 minimum cell voltage in 2mV steps. 124 = 2,48V
	int8_t min_cell_volt_num;		//#38 number of the cell with the lowest voltage
	int8_t rpm2_L;				//#39 RPM in 10 RPM steps. 300 = 3000rpm
	int8_t rpm2_H;				//#40
	int8_t general_error_number;	//#41 Voice error == 12. TODO: more docu
	int8_t pressure;				//#42 Pressure up to 16bar. 0,1bar scale. 20 = 2bar
	int8_t version;				//#43 version number TODO: more info?
	int8_t stop_byte;				//#44 stop int8_t
//	int8_t parity;				//#45 CRC/Parity
};

static struct HOTT_GAM_MSG hott_gam_msg;
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
#define HOTT_VARIO_MSG_TEXT_LEN	21

struct HOTT_VARIO_MSG {
	int8_t start_byte;			//#01 start int8_t constant value 0x7c
	int8_t vario_sensor_id; 		//#02 VARIO sensort id. constat value 0x89
	int8_t warning_beeps;			//#03 1=A 2=B ...
								// Q	Min cell voltage sensor 1
								// R	Min Battery 1 voltage sensor 1
								// J	Max Battery 1 voltage sensor 1
								// F	Min temperature sensor 1
								// H	Max temperature sensor 1
								// S	Min Battery voltage sensor 2
								// K	Max Battery voltage sensor 2
								// G	Min temperature sensor 2
								// I	Max temperature sensor 2
								// W	Max current
								// V	Max capacity mAh
								// P	Min main power voltage
								// X	Max main power voltage
								// O	Min altitude
								// Z	Max altitude
								// T	Minimum RPM
								// Y	Maximum RPM
								// C	m/s negative difference
								// A	m/3s negative difference


	int8_t sensor_id;	            //#04 constant value 0x90
	int8_t alarm_invers1;		//#05 Inverse display (alarm?) bitmask
								//TODO: more info
	int8_t altitude_L;			//#06 Altitude low int8_t. In meters. A value of 500 means 0m
	int8_t altitude_H;			//#07 Altitude high int8_t
	int8_t altitude_max_L;		//#08 Max. measured altitude low int8_t. In meters. A value of 500 means 0m
	int8_t altitude_max_H;		//#09 Max. measured altitude high int8_t
	int8_t altitude_min_L;		//#10 Min. measured altitude low int8_t. In meters. A value of 500 means 0m
	int8_t altitude_min_H;		//#11 Min. measured altitude high int8_t
	int8_t climbrate_L;			//#12 Climb rate in m/s. Steps of 0.01m/s. Value of 30000 = 0.00 m/s
	int8_t climbrate_H;			//#13 Climb rate in m/s
	int8_t climbrate3s_L;			//#14 Climb rate in m/3s. Steps of 0.01m/3s. Value of 30000 = 0.00 m/3s
	int8_t climbrate3s_H;			//#15 Climb rate m/3s low int8_t
	int8_t climbrate10s_L;		//#16 Climb rate m/10s. Steps of 0.01m/10s. Value of 30000 = 0.00 m/10s
	int8_t climbrate10s_H;		//#17 Climb rate m/10s low int8_t
	int8_t text_msg[HOTT_VARIO_MSG_TEXT_LEN];			//#18 Free ASCII text message
	int8_t free_char1;			//#39 Free ASCII character.  appears right to home distance
	int8_t free_char2;			//#40 Free ASCII character.  appears right to home direction
	int8_t free_char3;			//#41 Free ASCII character.  appears? TODO: Check where this char appears
	int8_t compass_direction;		//#42 Compass heading in 2° steps. 1 = 2°
	int8_t version;				//#43 version number TODO: more info?
	int8_t stop_byte;				//#44 stop int8_t, constant value 0x7d
//	int8_t parity;				//#45 checksum / parity
};

static HOTT_VARIO_MSG hott_vario_msg;
#endif

#ifdef HOTT_SIM_EAM_SENSOR
struct HOTT_EAM_MSG {
	int8_t start_byte;			//#01 start int8_t
	int8_t eam_sensor_id; 		//#02 EAM sensort id. constat value 0x8e
	int8_t warning_beeps;			//#03 1=A 2=B ... or 'A' - 0x40 = 1
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
								
	int8_t sensor_id;	            //#04 constant value 0xe0
	int8_t alarm_invers1;			//#05 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm field
								// 0	mAh
								// 1	Battery 1
								// 2	Battery 2
								// 3	Temperature 1
								// 4	Temperature 2
								// 5	Altitude
								// 6	Current
								// 7	Main power voltage
	int8_t alarm_invers2;			//#06 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm Field
								// 0	m/s
								// 1	m/3s
								// 2	Altitude (duplicate?)
								// 3	m/s	(duplicate?)					
								// 4	m/3s (duplicate?)
								// 5	unknown/unused
								// 6	unknown/unused
								// 7	"ON" sign/text msg active
								
	int8_t cell1_L;				//#07 cell 1 voltage lower value. 0.02V steps, 124=2.48V
	int8_t cell2_L;				//#08
	int8_t cell3_L;				//#09
	int8_t cell4_L;				//#10
	int8_t cell5_L;				//#11
	int8_t cell6_L;				//#12
	int8_t cell7_L;				//#13
	int8_t cell1_H;				//#14 cell 1 voltage high value. 0.02V steps, 124=2.48V
	int8_t cell2_H;				//#15
	int8_t cell3_H;				//#16
	int8_t cell4_H;				//#17
	int8_t cell5_H;				//#18
	int8_t cell6_H;				//#19
	int8_t cell7_H;				//#20
	
	int8_t batt1_voltage_L;		//#21 battery 1 voltage lower value. opt. cell8_L 0.02V steps
	int8_t batt1_voltage_H;		//#22
	
	int8_t batt2_voltage_L;		//#23 battery 2 voltage lower value. opt cell8_H value-. 0.02V steps
	int8_t batt2_voltage_H;		//#24
	
	int8_t temp1;					//#25 Temperature sensor 1. 0°=20, 26°=46
	int8_t temp2;					//#26 temperature sensor 2
	
	int8_t altitude_L;			//#27 Attitude lower value. unit: meters. Value of 500 = 0m
	int8_t altitude_H;			//#28
	
	int8_t current_L;				//#29 Current in 0.1 steps
	int8_t current_H;				//#30
	
	int8_t main_voltage_L;		//#30 Main power voltage (drive) in 0.1V steps
	int8_t main_voltage_H;		//#31
	
	int8_t batt_cap_L;			//#32 used battery capacity in 10mAh steps
	int8_t batt_cap_H;			//#33
	
	int8_t climbrate_L;			//#34 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
	int8_t climbrate_H;			//#35
	
	int8_t climbrate3s;			//#36 climbrate in m/3sec. Value of 120 = 0m/3sec
	
	int8_t rpm_L;					//#37 RPM. Steps: 10 U/min
	int8_t rpm_H;					//#38
	
	int8_t electric_min;			//#39 Electric minutes. Time does start, when motor current is > 3 A
	int8_t electric_sec;			//#40
	
	int8_t speed_L;				//#41 (air?) speed in km/h. Steps 1km/h
	int8_t speed_H;				//#42
	int8_t stop_byte;				//#43 stop int8_t
//	int8_t parity;				//#44 CRC/Parity
};

static HOTT_EAM_MSG hott_eam_msg;
#endif

#ifdef HOTT_SIM_GPS_SENSOR
//HoTT GPS Sensor response to Receiver (?!not?! Smartbox)
struct HOTT_GPS_MSG {
  int8_t start_byte;      //#01 constant value 0x7c
  int8_t gps_sensor_id;   //#02 constant value 0x8a
  int8_t warning_beeps;   //#03 1=A 2=B ...
						// A	Min Speed
						// L	Max Speed
						// O	Min Altitude
						// Z	Max Altitude
						// C	(negative) sink rate m/sec to high
						// B	(negative) sink rate m/3sec to high
						// N	climb rate m/sec to high
						// M	climb rate m/3sec to high
						// D	Max home distance
						// 

  int8_t sensor_id;       //#04 constant (?) value 0xa0
  int8_t alarm_invers1;	//#05
  						//TODO: more info
  int8_t alarm_invers2;	//#06  1 = No GPS signal
  								//TODO: more info

  int8_t flight_direction; //#07 flight direction in 2 degreees/step (1 = 2degrees);
  int8_t gps_speed_L;  //08 km/h
  int8_t gps_speed_H;  //#09
  
  int8_t pos_NS;  //#10 north = 0, south = 1
  int8_t pos_NS_dm_L;  //#11 degree minutes ie N48°39’988
  int8_t pos_NS_dm_H;  //#12
  int8_t pos_NS_sec_L; //#13 position seconds
  int8_t pos_NS_sec_H;  //#14

  int8_t pos_EW;  //#15 east = 0, west = 1
  int8_t pos_EW_dm_L; //#16 degree minutes ie. E9°25’9360
  int8_t pos_EW_dm_H;  //#17
  int8_t pos_EW_sec_L;  //#18 position seconds
  int8_t pos_EW_sec_H;  //#19
  
  int8_t home_distance_L;  //#20 meters
  int8_t home_distance_H;  //#21
  
  int8_t altitude_L; //#22 meters. Value of 500 = 0m 
  int8_t altitude_H; //#23
  
  int8_t climbrate_L;  //#24 m/s 0.01m/s resolution. Value of 30000 = 0.00 m/s
  int8_t climbrate_H;  //#25
  
  int8_t climbrate3s;  //#26 climbrate in m/3s resolution, value of 120 = 0 m/3s
  
  int8_t gps_satelites;  //#27 sat count
  int8_t gps_fix_char; //#28 GPS fix character. display, 'D' = DGPS, '2' = 2D, '3' = 3D, '-' = no fix. Where appears this char???
  
  int8_t home_direction; //#29 direction from starting point to Model position (2 degree steps)
  int8_t angle_roll;    //#30 angle roll in 2 degree steps
  int8_t angle_nick;    //#31 angle in 2degree steps
  int8_t angle_compass; //#32 angle in 2degree steps. 1 = 2°, 255 = - 2° (1 int8_t) North = 0°
  
  int8_t gps_time_h;  //#33 UTC time hours
  int8_t gps_time_m;  //#34 UTC time minutes
  int8_t gps_time_s;  //#35 UTC time seconds
  int8_t gps_time_sss;//#36 UTC time milliseconds
  
  int8_t msl_altitude_L;  //#37 mean sea level altitude
  int8_t msl_altitude_H;  //#38
  
  int8_t vibration; //#39 vibrations level in %
  
  int8_t free_char1;  //#40 appears right to home distance
  int8_t free_char2;  //#41 appears right to home direction
  int8_t free_char3;  //#42 GPS ASCII D=DGPS 2=2D 3=3D -=No Fix
  int8_t version;  //#43
  				 // 0	GPS Graupner #33600
                 // 1	Gyro Receiver
                 // 255 Mikrokopter
  int8_t end_byte;  //#44 constant value 0x7d
//  int8_t parity;    //#45
};

static HOTT_GPS_MSG hott_gps_msg;
#endif

// HoTT serial send buffer pointer
static int8_t *_hott_msg_ptr = 0;
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
#endif

#if HOTT_TELEMETRY_SERIAL_PORT == 3
void _hott_enable_transmitter() {
  //enables serial transmitter, disables receiver
    UCSR3B &= ~_BV(RXEN3);
    UCSR3B |= _BV(TXEN3); 
}

void _hott_enable_receiver() {
   //enables serial receiver, disables transmitter
  UCSR3B &= ~_BV(TXEN3);
  UCSR3B |= _BV(RXEN3);
}
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
#endif

#ifdef HOTT_SIM_EAM_SENSOR
 memset(&hott_eam_msg, 0, sizeof(struct HOTT_EAM_MSG));
 hott_eam_msg.start_byte = 0x7c;
 hott_eam_msg.eam_sensor_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
 hott_eam_msg.sensor_id = 0xe0;
 hott_eam_msg.stop_byte = 0x7d;
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
 
#ifdef HOTT_SIM_TEXTMODE
  memset(&hott_txt_msg, 0, sizeof(struct HOTT_TEXTMODE_MSG));
  hott_txt_msg.start_byte = 0x7b;
  hott_txt_msg.fill1 = 0x00;
  hott_txt_msg.warning_beeps = 0x00;
  hott_txt_msg.stop_byte = 0x7d;
#endif
}

/*
  called by timer_scheduler
*/
/*
  Called periodically (≈1ms) by timer scheduler
  
  @param tnow  timestamp in uSecond
*/
void _hott_serial_scheduler(uint32_t tnow) {
	static uint32_t _hott_serial_timer;
   _hott_check_serial_data(tnow);	// Check for data request
  if(_hott_msg_ptr == 0) return;   //no data to send
  if(_hott_telemetry_is_sending) {
    //we are sending already, wait for a delay of 2ms between data bytes
    if(tnow - _hott_serial_timer < 3000)  //delay ca. 3,5 mS. 19200 baud = 520uS / int8_t + 3ms required delay
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
  static int msg_crc = 0;
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
    msg_crc = 0;
    _hott_enable_receiver();
    _HOTT_PORT->flush();
  } else {
    --_hott_msg_len;
    if(_hott_msg_len != 0) {
       	msg_crc += *_hott_msg_ptr; 
	    _HOTT_PORT->write(*_hott_msg_ptr++);
    } else {
    	//last int8_t, send crc
	    _HOTT_PORT->write((int8_t) (msg_crc));
    }
  }
}

/*
  Onetime setup for HoTT
*/
void _hott_setup() { 
  _HOTT_PORT->begin(19200);
  _hott_enable_receiver();
//check AVR_SCHEDULER_MAX_TIMER_PROCS
	hal.scheduler->register_timer_process(_hott_serial_scheduler);
  //init msgs
  _hott_msgs_init();  //set default values  
}

/*
  Check for a valid HoTT requests on serial bus
*/

void _hott_check_serial_data(uint32_t tnow) {
	static uint32_t _hott_serial_request_timer = 0;
	if(_hott_telemetry_is_sending == true) return;
    if(_HOTT_PORT->available() > 1) {
      if(_HOTT_PORT->available() == 2) {
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
        unsigned char c = _HOTT_PORT->read();
        unsigned char addr = _HOTT_PORT->read();
        //ok, we have a valid request, check for address
        switch(c) {
//*****************************************************************************
#ifdef HOTT_SIM_TEXTMODE
            case HOTT_TEXT_MODE_REQUEST_ID:
           //Text mode, handle only if not armed!
           if(!motors.armed())
           {
				hott_txt_msg.start_byte = 0x7b;
				hott_txt_msg.stop_byte = 0x7d;
				uint8_t tmp = (addr >> 4);  // Sensor type
				if(tmp == (HOTT_SIM_TEXTMODE_ADDRESS & 0x0f))   {
					HOTT_Clear_Text_Screen();
					HOTT_HandleTextMode(addr); 
					_hott_send_text_msg();   //send message
				}
           }
           break;
#endif
//*****************************************************************************
          case HOTT_BINARY_MODE_REQUEST_ID:
//          cliSerial->printf_P(PSTR("\nHott\n"));
#ifdef HOTT_SIM_GPS_SENSOR
			//GPS module binary mode
            if(addr == HOTT_TELEMETRY_GPS_SENSOR_ID) {
              _hott_send_gps_msg();
              break;
            }
#endif
#ifdef HOTT_SIM_EAM_SENSOR
            if(addr == HOTT_TELEMETRY_EAM_SENSOR_ID) {
		      _hott_send_eam_msg();
              break;
		    }
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
            if(addr == HOTT_TELEMETRY_VARIO_SENSOR_ID) {
		      _hott_send_vario_msg();
              break;
		    }
#endif
#ifdef HOTT_SIM_GAM_SENSOR
            if(addr == HOTT_TELEMETRY_GAM_SENSOR_ID) {
		      _hott_send_gam_msg();
              break;
		    }
#endif

//END: case HOTT_BINARY_MODE_REQUEST_ID:
           break;
//*****************************************************************************         
          default:
            break;
        }
      } else {
        //ignore data from other sensors
        _HOTT_PORT->flush();
        _hott_serial_request_timer = 0;
      }
    }
}

void _hott_send_msg(int8_t *buffer, int len) {
  if(_hott_telemetry_is_sending == true) return;
  _hott_msg_ptr = buffer;
  _hott_msg_len = len +1; //len + 1 byte for crc
}

#ifdef HOTT_SIM_GAM_SENSOR
void _hott_send_gam_msg() {
	_hott_send_msg((int8_t *)&hott_gam_msg, sizeof(struct HOTT_GAM_MSG));
  _hott_telemetry_sendig_msgs_id = HOTT_TELEMETRY_GAM_SENSOR_ID;
}
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
void _hott_send_vario_msg() {
	_hott_send_msg((int8_t *)&hott_vario_msg, sizeof(struct HOTT_VARIO_MSG));
	_hott_telemetry_sendig_msgs_id = HOTT_TELEMETRY_VARIO_SENSOR_ID;
}
#endif

#ifdef HOTT_SIM_GPS_SENSOR
void _hott_send_gps_msg() {
  _hott_send_msg((int8_t *)&hott_gps_msg, sizeof(struct HOTT_GPS_MSG));
  _hott_telemetry_sendig_msgs_id = HOTT_TELEMETRY_GPS_SENSOR_ID;
}
#endif

#ifdef HOTT_SIM_EAM_SENSOR
//Send EMA sensor data
void _hott_send_eam_msg() {
  _hott_send_msg((int8_t *)&hott_eam_msg, sizeof(struct HOTT_EAM_MSG));
  _hott_telemetry_sendig_msgs_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
}
#endif

#ifdef HOTT_SIM_TEXTMODE
void _hott_send_text_msg() {
  _hott_send_msg((int8_t *)&hott_txt_msg, sizeof(struct HOTT_TEXTMODE_MSG));
  _hott_telemetry_sendig_msgs_id = HOTT_TEXT_MODE_REQUEST_ID;
}
#endif

#ifdef HOTT_SIM_GAM_SENSOR
void _hott_update_gam_msg() {
	hott_gam_msg.temperature1 = (int8_t)((barometer.get_temperature() / 10) + 20);
	hott_gam_msg.temperature2 = 20; // 0°C
	(int &)hott_gam_msg.altitude_L = (int)((current_loc.alt - home.alt) / 100)+500;
	(int &)hott_gam_msg.climbrate_L = 30000 + climb_rate;
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
}
#endif

#ifdef HOTT_SIM_EAM_SENSOR
//Update EAM sensor data
void _hott_update_eam_msg() {
	(int &)hott_eam_msg.batt1_voltage_L = (int)(0);
	(int &)hott_eam_msg.batt2_voltage_L = (int)(0);
	hott_eam_msg.temp1 = (int8_t)((barometer.get_temperature() / 10) + 20);
	hott_eam_msg.temp2 = 20;	//0°
	(int &)hott_eam_msg.altitude_L = (int)((current_loc.alt - home.alt) / 100)+500;
	(int &)hott_eam_msg.current_L = current_amps1*10;
	(int &)hott_eam_msg.main_voltage_L = (int)(battery_voltage1 * 10.0);
	(int &)hott_eam_msg.batt_cap_L = current_total1 / 10;
	(int &)hott_eam_msg.speed_L = (int)((float)(g_gps->ground_speed * 0.036));

  	(int &)hott_eam_msg.climbrate_L = 30000 + climb_rate;  
  	hott_eam_msg.climbrate3s = 120 + (climb_rate / 100);  // 0 m/3s using filtered data here
  	
    //display ON when motors are armed
    if (motors.armed()) {
       hott_eam_msg.alarm_invers2 |= 0x80;
     } else {
       hott_eam_msg.alarm_invers2 &= 0x7f;
     }
}

#endif

#ifdef HOTT_SIM_GPS_SENSOR
// Updates GPS message values
void _hott_update_gps_msg() { 
  // update GPS telemetry data
  (int &)hott_gps_msg.msl_altitude_L = (int)g_gps->altitude / 100;  //meters above sea level  
  hott_gps_msg.flight_direction = (int8_t)(g_gps->ground_course / 200);  // in 2* steps
  (int &)hott_gps_msg.gps_speed_L = (int)((float)(g_gps->ground_speed * 0.036));

  
  if(g_gps->status() == GPS::GPS_OK_FIX_3D) {
    hott_gps_msg.alarm_invers2 = 0;
    hott_gps_msg.gps_fix_char = '3';  
    hott_gps_msg.free_char3 = '3';  //3D Fix according to specs...
  } else {
    //No GPS Fix
    hott_gps_msg.alarm_invers2 = 1;
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
    	      hott_gps_msg.home_direction = get_bearing_cd(&current_loc, &next_WP) / 200; //get_bearing() return value in degrees * 100
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
          hott_gps_msg.home_direction = get_bearing_cd(&current_loc, &home) / 200; //get_bearing() return value in degrees * 100
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

  (int &)hott_gps_msg.climbrate_L = 30000 + climb_rate;  
  hott_gps_msg.climbrate3s = 120 + (climb_rate / 100);  // 0 m/3s
  
  hott_gps_msg.gps_satelites = (int8_t)g_gps->num_sats;
  
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
}
#endif

#ifdef HOTT_SIM_VARIO_SENSOR

const char hott_flight_mode_strings[NUM_MODES+1][10] PROGMEM = {
    "STABILIZE",        		// 0
    "ACRO",                     // 1
    "ALT_HOLD",                 // 2
    "AUTO",                     // 3
    "GUIDED",                   // 4
    "LOITER",                   // 5
    "RTL",                      // 6
    "CIRCLE",                   // 7
    "POSITION",                 // 8
    "LAND",                     // 9
    "OF_LOITER",        		// 10
    "TOY_M",                    // 11
    "TOY_A",					// 12
    "???"
};

const char hott_ARMED_STR[] PROGMEM	= "ARMED";
const char hott_DISARMED_STR[] PROGMEM	= "DISARMED";

void _hott_update_vario_msg() {
	static int _hott_max_altitude = 0;
	static int _hott_min_altitude = 0;

	(int &)hott_vario_msg.altitude_L = (int)((current_loc.alt - home.alt) / 100)+500;

	if( (current_loc.alt - home.alt) > _hott_max_altitude)
		_hott_max_altitude = (current_loc.alt - home.alt);
	(int &)hott_vario_msg.altitude_max_L = (int)(_hott_max_altitude / 100)+500;

	if( (current_loc.alt - home.alt) < _hott_min_altitude)
		_hott_min_altitude = (current_loc.alt - home.alt);
	(int &)hott_vario_msg.altitude_min_L = (int)(_hott_min_altitude / 100)+500;
	
	(int &)hott_vario_msg.climbrate_L = 30000 + climb_rate;
	(int &)hott_vario_msg.climbrate3s_L = 30000 + climb_rate;	//using filtered data here
	(int &)hott_vario_msg.climbrate10s_L = 30000;	//TODO: calc this stuff
	
	hott_vario_msg.compass_direction = ToDeg(compass.calculate_heading(ahrs.get_dcm_matrix())) / 2;
	
	//Free text processing
	char mode[10];
	char armed[10];
        if(control_mode > NUM_MODES)
          control_mode = NUM_MODES;
	strcpy_P(mode,hott_flight_mode_strings[control_mode]);
	
	if (motors.armed()) {
          strcpy_P(armed,hott_ARMED_STR);
	} else {
          strcpy_P(armed,hott_DISARMED_STR);
	}
	memset(hott_vario_msg.text_msg,0x20,HOTT_VARIO_MSG_TEXT_LEN);
	snprintf((char*)hott_vario_msg.text_msg,HOTT_VARIO_MSG_TEXT_LEN, "%s %s", &armed[0], &mode[0]);
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
		str[i] = (int8_t)(0x80 + (int8_t)str[i]);
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
	_hoot_check_alarm();
	_hott_alarm_scheduler();
	_hott_update_replay_queue();
}

//****************************************************************************************
// Alarm stuff
//
//HoTT alarm macro
#define HOTT_ALARM_NUM(a) (a-0x40)

struct _hott_alarm_event_T {
	uint16_t alarm_time; 		//Alarm play time in 1sec units
	uint16_t alarm_time_replay;	//Alarm repeat time in 1sec. units. 0 -> One time alarm
								//forces a delay between new alarms of the same kind
	uint8_t visual_alarm1;		//Visual alarm bitmask
	uint8_t visual_alarm2;		//Visual alarm bitmask
	uint8_t alarm_num;			//Alarm number 0..255 (A-Z)
	uint8_t alarm_profile;		//profile id ie HOTT_TELEMETRY_GPS_SENSOR_ID
};
typedef struct _hott_alarm_event_T _hott_alarm_event;

#define HOTT_ALARM_QUEUE_MAX		5
//TODO: better queueing solution
static _hott_alarm_event _hott_alarm_queue[HOTT_ALARM_QUEUE_MAX];
static _hott_alarm_event _hott_alarm_replay_queue[HOTT_ALARM_QUEUE_MAX];
static uint8_t _hott_alarmCnt = 0;
static uint8_t _hott_alarm_ReplayCnt = 0;

//
// checks if an alarm exists in active queue
//
bool _hoot_alarm_active_exists(struct _hott_alarm_event_T *alarm) {
	//check active alarms
	for(uint8_t i=0; i<_hott_alarmCnt; i++) {
		if(_hott_alarm_queue[i].alarm_num == alarm->alarm_num &&
			_hott_alarm_queue[i].alarm_profile == alarm->alarm_profile) {
			//alarm exists.
			return true;
		}
	}
	return false;
}

//
// checks if an alarm exists in replay queue
//
bool _hoot_alarm_replay_exists(struct _hott_alarm_event_T *alarm) {
	//check replay delay queue
	for(uint8_t i=0; i<_hott_alarm_ReplayCnt; i++) {
		if(_hott_alarm_replay_queue[i].alarm_num == alarm->alarm_num &&
			_hott_alarm_replay_queue[i].alarm_profile == alarm->alarm_profile) {
			//alarm exists
			return true;
		}
	}
	return false;
}

//
// checks if an alarm exists
//
bool _hoot_alarm_exists(struct _hott_alarm_event_T *alarm) {
	if(_hoot_alarm_active_exists(alarm))
		return true;
	if(_hoot_alarm_replay_exists(alarm))
		return true;
	return false;
}

//
// adds an alarm to active queue
//
void _hott_add_alarm(struct _hott_alarm_event_T *alarm) {
	if(alarm == 0)
		return;
	if(_hott_alarmCnt >= HOTT_ALARM_QUEUE_MAX)
		return;	//no more space left...
	if(_hoot_alarm_exists(alarm)) 
		return;
	// we have a new alarm
	memcpy(&_hott_alarm_queue[_hott_alarmCnt++], alarm, sizeof(struct _hott_alarm_event_T));
}

//
// adds an alarm to replay queue
//
void _hott_add_replay_alarm(struct _hott_alarm_event_T *alarm) {
	if(alarm == 0)
		return;
	if(_hott_alarm_ReplayCnt >= HOTT_ALARM_QUEUE_MAX)
		return;	//no more space left...
	if(_hoot_alarm_replay_exists(alarm)) 
		return;
	// we have a new alarm
	memcpy(&_hott_alarm_replay_queue[_hott_alarm_ReplayCnt++], alarm, sizeof(struct _hott_alarm_event_T));
}

//
//removes an alarm from active queue
//first alarm at offset 1
//
void _hott_remove_alarm(uint8_t num) {
	if(num > _hott_alarmCnt || num == 0)	//has to be > 0
		return; // possibile error

	if(_hott_alarmCnt != 1) {
		memcpy(&_hott_alarm_queue[num-1], &_hott_alarm_queue[num], sizeof(struct _hott_alarm_event_T) * (_hott_alarmCnt - num) );
	}
	--_hott_alarmCnt;
}

//
//removes an alarm from replay queue
//first alarm at offset 1
//
void _hott_remove_replay_alarm(uint8_t num) {
	if(num > _hott_alarm_ReplayCnt || num == 0)	//has to be > 0
		return; // possibile error

	if(_hott_alarm_ReplayCnt != 1) {
		memcpy(&_hott_alarm_replay_queue[num-1], &_hott_alarm_replay_queue[num], sizeof(struct _hott_alarm_event_T) * (_hott_alarm_ReplayCnt - num) );
	}
	--_hott_alarm_ReplayCnt;
}

//
// Updates replay delay queue
//
void _hott_update_replay_queue(void) {
static uint8_t t = 0;
	if(++t < 50)
		return;
	//every second
	t = 0;

	for(uint8_t i=0; i< _hott_alarm_ReplayCnt; i++) {
		if(--_hott_alarm_replay_queue[i].alarm_time_replay == 0) {
			//remove it
			_hott_remove_replay_alarm(i+1);
			i--;
			continue;
		}
	}
}

//
// Sets a voice alarm value
//
void _hott_set_voice_alarm(uint8_t profile, uint8_t value) {
	switch(profile) {
#ifdef HOTT_SIM_EAM_SENSOR
  		case HOTT_TELEMETRY_EAM_SENSOR_ID:
			hott_eam_msg.warning_beeps = value;
			break;
#endif
#ifdef HOTT_SIM_GPS_SENSOR
		case HOTT_TELEMETRY_GPS_SENSOR_ID:
			hott_gps_msg.warning_beeps = value;
			break;
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
		case HOTT_TELEMETRY_VARIO_SENSOR_ID:
			hott_vario_msg.warning_beeps = value;
			break;
#endif
#ifdef HOTT_SIM_GAM_SENSOR
		case HOTT_TELEMETRY_GAM_SENSOR_ID:
			hott_gam_msg.warning_beeps = value;
			break;
#endif
		default:
			break;
	}
}

//
// active alarm scheduler
//
void _hott_alarm_scheduler() {
	static uint8_t activeAlarmTimer = 3* 50;
	static uint8_t activeAlarm = 0;

	if(_hott_alarmCnt < 1)
		return;	//no alarms	

	uint8_t vEam = 0;
	uint8_t vEam2 = 0;
	uint8_t vVario = 0;
	uint8_t vGps = 0;
	uint8_t vGps2 = 0;
#ifdef HOTT_SIM_GAM_SENSOR
	uint8_t vGam = 0;
	uint8_t vGam2 = 0;
#endif
	
	for(uint8_t i = 0; i< _hott_alarmCnt; i++) {
		if(_hott_alarm_queue[i].alarm_time == 0) {
			//end of alarm, remove it
			//no check for current msg to be send, so maybe the crc will be wrong
			_hott_set_voice_alarm(_hott_alarm_queue[i].alarm_profile, 0);
			if(_hott_alarm_queue[i].alarm_time_replay != 0)
				_hott_add_replay_alarm(&_hott_alarm_queue[i]);
			_hott_remove_alarm(i+1);	//first alarm at offset 1
			--i;	//correct counter
			continue;
		}
		
		//
		switch(_hott_alarm_queue[i].alarm_profile) {
#ifdef HOTT_SIM_EAM_SENSOR
			case HOTT_TELEMETRY_EAM_SENSOR_ID:
				vEam |= _hott_alarm_queue[i].visual_alarm1;
				vEam2 |= _hott_alarm_queue[i].visual_alarm2;
				break;
#endif
#ifdef HOTT_SIM_GPS_SENSOR
			case HOTT_TELEMETRY_GPS_SENSOR_ID:
				vGps |= _hott_alarm_queue[i].visual_alarm1;
				vGps2 |= _hott_alarm_queue[i].visual_alarm2;
				break;
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
			case HOTT_TELEMETRY_VARIO_SENSOR_ID:
				vVario |= _hott_alarm_queue[i].visual_alarm1;
				break;
#endif
#ifdef HOTT_SIM_GAM_SENSOR
			case HOTT_TELEMETRY_GAM_SENSOR_ID:
				vGam |= _hott_alarm_queue[i].visual_alarm1;		
				vGam2 |= _hott_alarm_queue[i].visual_alarm2;		
				break;
#endif
			default:
				break;
		}
	} //end: visual alarm loop
#ifdef HOTT_SIM_EAM_SENSOR
	// Set all visual alarms
	hott_eam_msg.alarm_invers1 |= vEam;
	hott_eam_msg.alarm_invers2 |= vEam2;
#endif
#ifdef HOTT_SIM_GAM_SENSOR
	hott_gam_msg.alarm_invers1 |= vGam;
	hott_gam_msg.alarm_invers2 |= vGam2;
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
	hott_vario_msg.alarm_invers1 |= vVario;
#endif
#ifdef HOTT_SIM_GPS_SENSOR
	hott_gps_msg.alarm_invers1 |= vGps;
	hott_gps_msg.alarm_invers2 |= vGps2;
#endif
	if(activeAlarm != 0) { //is an alarm active
		if ( ++activeAlarmTimer % 50 == 0 ) {	//every 1sec
			_hott_alarm_queue[activeAlarm-1].alarm_time--;
		}
		if ( activeAlarmTimer < 50 * 2) //alter alarm every 2 sec
			return;
	}
	activeAlarmTimer = 0;

	if(++activeAlarm > _hott_alarmCnt) {
		activeAlarm = 1;
	}
	if(_hott_alarmCnt <= 0) {
		activeAlarm = 0;
		return;
	}
	_hott_set_voice_alarm(_hott_alarm_queue[activeAlarm-1].alarm_profile, _hott_alarm_queue[activeAlarm-1].alarm_num);
}

//****************************************************************************************
// Sensor specific code
//
#ifdef HOTT_SIM_EAM_SENSOR
//
// triggers max consumed mAh alarm
//
void _hott_eam_check_mAh() {
	_hott_alarm_event _hott_ema_alarm_event;
	if( (g.battery_monitoring == 4) && (g.pack_capacity <= (int16_t)current_total1)) {
		_hott_ema_alarm_event.alarm_time = 6;	//1sec units
		_hott_ema_alarm_event.alarm_time_replay = 15;	//1sec units
		_hott_ema_alarm_event.visual_alarm1 = 0x01;	//blink mAh
		_hott_ema_alarm_event.visual_alarm2 = 0;
		_hott_ema_alarm_event.alarm_num = HOTT_ALARM_NUM('V');
		_hott_ema_alarm_event.alarm_profile = HOTT_TELEMETRY_EAM_SENSOR_ID;
		_hott_add_alarm(&_hott_ema_alarm_event);
	}
}

//
// triggers low main power voltage alarm
//
void _hott_eam_check_mainPower() {
	//voltage sensor needs some time at startup
	if((millis() > 10000) && (g.battery_monitoring != 0) && (battery_voltage1 < g.low_voltage)) {
		_hott_alarm_event _hott_ema_alarm_event;
		_hott_ema_alarm_event.alarm_time = 6;	//1sec units
		_hott_ema_alarm_event.alarm_time_replay = 30; //1sec unit
		_hott_ema_alarm_event.visual_alarm1 = 0x80;	//blink main power
		_hott_ema_alarm_event.visual_alarm2 = 0;
		_hott_ema_alarm_event.alarm_num = HOTT_ALARM_NUM('P');
		_hott_ema_alarm_event.alarm_profile = HOTT_TELEMETRY_EAM_SENSOR_ID;
		_hott_add_alarm(&_hott_ema_alarm_event);
	}
}
#endif

//
//	alarm triggers to check
//
void _hoot_check_alarm()  {
#ifdef HOTT_SIM_EAM_SENSOR
	_hott_eam_check_mAh();
	_hott_eam_check_mainPower();
#endif
}


#endif	//End of #ifdef HOTT_TELEMETRY
