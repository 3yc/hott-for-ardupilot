/**
 * @file hott_msgs.h
 * @author Adam Majerczyk (majerczyk.adam@gmail.com)
*/

//
// Graupner HoTT v4 telemetry for ArduCopter on PX4 & Pixhawk hardware
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
//
//

#ifndef HOTT_MSGS_H
#define HOTT_MSGS_H

#define HOTT_TEXT_MODE_REQUEST_ID	0x7f
#define HOTT_BINARY_MODE_REQUEST_ID	0x80
//Sensor Ids

//Id 0x80 is used when no sensor has been found during the bus scan
// additionaly meaning?
#define HOTT_TELEMETRY_NO_SENSOR_ID		0x80

//Graupner #33601 Vario Module
#define HOTT_TELEMETRY_VARIO_SENSOR_ID	0x89

//Graupner #33600 GPS Module
#define HOTT_TELEMETRY_GPS_SENSOR_ID	0x8a

//Graupner #337xx Air ESC
#define HOTT_TELEMETRY_AIRESC_SENSOR_ID	0x8c

//Graupner #33611 General Air Module
#define HOTT_TELEMETRY_GAM_SENSOR_ID	0x8d

//Graupner #33620 Electric Air Module
#define HOTT_TELEMETRY_EAM_SENSOR_ID	0x8e


#define HOTT_TEXTMODE_MSG_TEXT_LEN 168
//Text mode msgs type
struct HOTT_TEXTMODE_MSG {
	int8_t start_byte;	//#01 constant value 0x7b
	int8_t fill1;		//#02 constant value 0x00
	int8_t warning_beeps;//#03 1=A 2=B ...
	int8_t msg_txt[HOTT_TEXTMODE_MSG_TEXT_LEN];	//#04 ASCII text to display to
						// Bit 7 = 1 -> Inverse character display
                        // Display 21x8
	int8_t stop_byte;	//#171 constant value 0x7d
	int8_t parity;		//#172 Checksum / parity
};
struct HOTT_GAM_MSG {
	int8_t start_byte;			//#01 start int8_t constant value 0x7c
	int8_t gam_sensor_id; 		//#02 EAM sensort id. constat value 0x8d
	int8_t warning_beeps;		//#03 1=A 2=B ... 0x1a=Z  0 = no alarm
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

	int8_t sensor_id;	        //#04 constant value 0xd0
	int8_t alarm_invers1;		//#05 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm field
								// 0	all cell voltage
								// 1	Battery 1
								// 2	Battery 2
								// 3	Temperature 1
								// 4	Temperature 2
								// 5	Fuel
								// 6	mAh
								// 7	Altitude
	int8_t alarm_invers2;		//#06 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm Field
								// 0	main power current
								// 1	main power voltage
								// 2	Altitude
								// 3	m/s				
								// 4	m/3s
								// 5	unknown
								// 6	unknown
								// 7	"ON" sign/text msg active
								
	int8_t cell1;				//#07 cell 1 voltage lower value. 0.02V steps, 124=2.48V
	int8_t cell2;				//#08
	int8_t cell3;				//#09
	int8_t cell4;				//#10
	int8_t cell5;				//#11
	int8_t cell6;				//#12
	int8_t batt1_L;				//#13 battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V
	int8_t batt1_H;				//#14
	int8_t batt2_L;				//#15 battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V
	int8_t batt2_H;				//#16
	int8_t temperature1;		//#17 temperature 1. offset of 20. a value of 20 = 0°C
	int8_t temperature2;		//#18 temperature 2. offset of 20. a value of 20 = 0°C
	int8_t fuel_procent;		//#19 Fuel capacity in %. Values 0--100
								// graphical display ranges: 0-25% 50% 75% 100%
	int8_t fuel_ml_L;			//#20 Fuel in ml scale. Full = 65535!
	int8_t fuel_ml_H;			//#21
	int8_t rpm_L;				//#22 RPM in 10 RPM steps. 300 = 3000rpm
	int8_t rpm_H;				//#23
	int8_t altitude_L;			//#24 altitude in meters. offset of 500, 500 = 0m
	int8_t altitude_H;			//#25
	int8_t climbrate_L;			//#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
	int8_t climbrate_H;			//#27
	int8_t climbrate3s;			//#28 climb rate in m/3sec. Value of 120 = 0m/3sec
	int8_t current_L;			//#29 current in 0.1A steps
	int8_t current_H;			//#30
	int8_t main_voltage_L;		//#31 Main power voltage using 0.1V steps
	int8_t main_voltage_H;		//#32
	int8_t batt_cap_L;			//#33 used battery capacity in 10mAh steps
	int8_t batt_cap_H;			//#34
	int8_t speed_L;				//#35 (air?) speed in km/h(?) we are using ground speed here per default
	int8_t speed_H;				//#36
	int8_t min_cell_volt;		//#37 minimum cell voltage in 2mV steps. 124 = 2,48V
	int8_t min_cell_volt_num;	//#38 number of the cell with the lowest voltage
	int8_t rpm2_L;				//#39 RPM in 10 RPM steps. 300 = 3000rpm
	int8_t rpm2_H;				//#40
	int8_t general_error_number;//#41 Voice error == 12. TODO: more docu
	int8_t pressure;			//#42 Pressure up to 16bar. 0,1bar scale. 20 = 2bar
	int8_t version;				//#43 version number TODO: more info?
	int8_t stop_byte;			//#44 stop int8_t
	int8_t parity;				//#45 CRC/Parity
};

#define HOTT_VARIO_MSG_TEXT_LEN	21
struct HOTT_VARIO_MSG {
	int8_t start_byte;			//#01 start int8_t constant value 0x7c
	int8_t vario_sensor_id; 	//#02 VARIO sensort id. constat value 0x89
	int8_t warning_beeps;		//#03 1=A 2=B ...
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


	int8_t sensor_id;	        //#04 constant value 0x90
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
	int8_t climbrate3s_L;		//#14 Climb rate in m/3s. Steps of 0.01m/3s. Value of 30000 = 0.00 m/3s
	int8_t climbrate3s_H;		//#15 Climb rate m/3s low int8_t
	int8_t climbrate10s_L;		//#16 Climb rate m/10s. Steps of 0.01m/10s. Value of 30000 = 0.00 m/10s
	int8_t climbrate10s_H;		//#17 Climb rate m/10s low int8_t
	int8_t text_msg[HOTT_VARIO_MSG_TEXT_LEN]; //#18 Free ASCII text message
	int8_t free_char1;			//#39 Free ASCII character.  appears right to home distance
	int8_t free_char2;			//#40 Free ASCII character.  appears right to home direction
	int8_t free_char3;			//#41 Free ASCII character.  appears? TODO: Check where this char appears
	int8_t compass_direction;	//#42 Compass heading in 2° steps. 1 = 2°
	int8_t version;				//#43 version number TODO: more info?
	int8_t stop_byte;			//#44 stop int8_t, constant value 0x7d
	int8_t parity;				//#45 checksum / parity
};

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
								
	int8_t sensor_id;	        //#04 constant value 0xe0
	int8_t alarm_invers1;		//#05 alarm bitmask. Value is displayed inverted
								//Bit#	Alarm field
								// 0	mAh
								// 1	Battery 1
								// 2	Battery 2
								// 3	Temperature 1
								// 4	Temperature 2
								// 5	Altitude
								// 6	Current
								// 7	Main power voltage
	int8_t alarm_invers2;		//#06 alarm bitmask. Value is displayed inverted
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
	
	int8_t temp1;				//#25 Temperature sensor 1. 0°=20, 26°=46
	int8_t temp2;				//#26 temperature sensor 2
	
	int8_t altitude_L;			//#27 Attitude lower value. unit: meters. Value of 500 = 0m
	int8_t altitude_H;			//#28
	
	int8_t current_L;			//#29 Current in 0.1 steps
	int8_t current_H;			//#30
	
	int8_t main_voltage_L;		//#30 Main power voltage (drive) in 0.1V steps
	int8_t main_voltage_H;		//#31
	
	int8_t batt_cap_L;			//#32 used battery capacity in 10mAh steps
	int8_t batt_cap_H;			//#33
	
	int8_t climbrate_L;			//#34 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
	int8_t climbrate_H;			//#35
	
	int8_t climbrate3s;			//#36 climbrate in m/3sec. Value of 120 = 0m/3sec
	
	int8_t rpm_L;				//#37 RPM. Steps: 10 U/min
	int8_t rpm_H;				//#38
	
	int8_t electric_min;		//#39 Electric minutes. Time does start, when motor current is > 3 A
	int8_t electric_sec;		//#40
	
	int8_t speed_L;				//#41 (air?) speed in km/h. Steps 1km/h
	int8_t speed_H;				//#42
	int8_t stop_byte;			//#43 stop int8_t
	int8_t parity;				//#44 CRC/Parity
};

//HoTT GPS Sensor response to Receiver (?!not?! Smartbox)
struct HOTT_GPS_MSG {
  int8_t start_byte;    //#01 constant value 0x7c
  int8_t gps_sensor_id; //#02 constant value 0x8a
  int8_t warning_beeps; //#03 1=A 2=B ...
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

  int8_t sensor_id;     //#04 constant (?) value 0xa0
  int8_t alarm_invers1;	//#05
  						//TODO: more info
  int8_t alarm_invers2;	//#06  1 = No GPS signal
  						//TODO: more info

  int8_t flight_direction; //#07 flight direction in 2 degreees/step (1 = 2degrees);
  int8_t gps_speed_L;  	//08 km/h
  int8_t gps_speed_H;	//#09
  
  int8_t pos_NS;  		//#10 north = 0, south = 1
  int8_t pos_NS_dm_L;  	//#11 degree minutes ie N48°39’988
  int8_t pos_NS_dm_H;  	//#12
  int8_t pos_NS_sec_L; 	//#13 position seconds
  int8_t pos_NS_sec_H;  //#14

  int8_t pos_EW;  		//#15 east = 0, west = 1
  int8_t pos_EW_dm_L; 	//#16 degree minutes ie. E9°25’9360
  int8_t pos_EW_dm_H; 	//#17
  int8_t pos_EW_sec_L;  //#18 position seconds
  int8_t pos_EW_sec_H;  //#19
  
  int8_t home_distance_L;  //#20 meters
  int8_t home_distance_H;  //#21
  
  int8_t altitude_L; 	//#22 meters. Value of 500 = 0m 
  int8_t altitude_H;	//#23
  
  int8_t climbrate_L;  	//#24 m/s 0.01m/s resolution. Value of 30000 = 0.00 m/s
  int8_t climbrate_H; 	 //#25
  
  int8_t climbrate3s;  	//#26 climbrate in m/3s resolution, value of 120 = 0 m/3s
  
  uint8_t gps_satelites;//#27 sat count
  uint8_t gps_fix_char; //#28 GPS fix character. display, 'D' = DGPS, '2' = 2D, '3' = 3D, '-' = no fix. Where appears this char???
  
  int8_t home_direction;//#29 direction from starting point to Model position (2 degree steps)
  int8_t angle_roll;    //#30 angle roll in 2 degree steps
  int8_t angle_nick;    //#31 angle in 2degree steps
  int8_t angle_compass; //#32 angle in 2degree steps. 1 = 2°, 255 = - 2° (1 int8_t) North = 0°
  
  int8_t gps_time_h;  	//#33 UTC time hours
  int8_t gps_time_m;  	//#34 UTC time minutes
  int8_t gps_time_s;  	//#35 UTC time seconds
  int8_t gps_time_sss;	//#36 UTC time milliseconds
  
  int8_t msl_altitude_L;//#37 mean sea level altitude
  int8_t msl_altitude_H;//#38
  
  int8_t vibration; 	//#39 vibrations level in %
  
  int8_t free_char1;  	//#40 appears right to home distance
  int8_t free_char2;  	//#41 appears right to home direction
  int8_t free_char3;  	//#42 GPS ASCII D=DGPS 2=2D 3=3D -=No Fix
  int8_t version;  		//#43
  				 		// 0	GPS Graupner #33600
                 		// 1	Gyro Receiver
                 		// 255 Mikrokopter
  int8_t end_byte;  	//#44 constant value 0x7d
  int8_t parity;    	//#45
};

struct HOTT_AIRESC_MSG {
    uint8_t start_byte;      //#01 constant value 0x7c
    uint8_t gps_sensor_id;   //#02 constant value 0x8c
    uint8_t warning_beeps;   //#03 1=A 2=B ...
							// A
							// L
							// O
							// Z
							// C
							// B
							// N
							// M
							// D
							// 

    uint8_t sensor_id;    	//#04 constant value 0xc0
    uint8_t alarm_invers1;	//#05 TODO: more info
    uint8_t alarm_invers2;	//#06 TODO: more info
	uint8_t input_v_L;		//#07 Input voltage low byte
	uint8_t input_v_H;		//#08
	uint8_t input_v_min_L;	//#09 Input min. voltage low byte
	uint8_t input_v_min_H;	//#10
	uint8_t batt_cap_L;		//#11 battery capacity in 10mAh steps
	uint8_t batt_cap_H;		//#12
	uint8_t	esc_temp;		//#13 ESC temperature
	uint8_t esc_max_temp;	//#14 ESC max. temperature
	uint8_t current_L;		//#15 Current in 0.1 steps
	uint8_t current_H;		//#16
	uint8_t	current_max_L;	//#17 Current max. in 0.1 steps
	uint8_t current_max_H;	//#18
	uint8_t rpm_L;			//#19 RPM in 10U/min steps
	uint8_t	rpm_H;			//#20
	uint8_t	rpm_max_L;		//#21 RPM max
	uint8_t rpm_max_H;		//#22
	uint8_t throttle;		//#23 throttle in %
	uint8_t speed_L;		//#24 Speed
	uint8_t speed_H;		//#25
	uint8_t	speed_max_L;	//#26 Speed max
	uint8_t speed_max_H;	//#27
	uint8_t bec_v;			//#28 BEC voltage
	uint8_t bec_min_v;		//#29 BEC min. voltage
	uint8_t bec_current;	//#30 BEC current
	uint8_t bec_current_max_L;	//#31 BEC max. current
	uint8_t bec_current_max_H;	//#32 TODO: not really clear why 2 bytes...
	uint8_t pwm;			//#33 PWM
	uint8_t bec_temp;		//#34 BEC temperature
	uint8_t bec_temp_max;	//#35 BEC highest temperature
	uint8_t motor_temp;		//#36 Motor or external sensor temperature
	uint8_t motor_temp_max;	//#37 Highest motor or external sensor temperature
	uint8_t motor_rpm_L;	//#38 Motor or external RPM sensor (without gear)
	uint8_t motor_rpm_H;	//#39
	uint8_t motor_timing;	//#40 Motor timing
	uint8_t motor_timing_adv; //#41 Motor advanced timing
	uint8_t motor_highest_current;	//#42 Motor number (1-x) with highest current
	uint8_t version;		//#43	Version number (highest current motor 1-x)
	uint8_t stop_byte;		//#44 constant value 0x7d
	uint8_t parity;			//#45 CRC/Parity
};

#endif