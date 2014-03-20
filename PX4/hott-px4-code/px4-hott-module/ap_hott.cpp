/**
 * @file ap_hott.cpp
 * @author Adam Majerczyk (majerczyk.adam@gmail.com)
 * @version 0.10.0.0b (beta)
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
// Uses parts of code from PX4 HoTT driver by Simon Wilks <sjwilks@gmail.com>
//
//
//
 
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <poll.h>

#include <sys/ioctl.h>

#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/ap_data.h>

#include "hott_msgs.h"

//#define DEFAULT_UART "/dev/ttyS0";		/**< USART1 */
//#define DEFAULT_UART "/dev/ttyS2";		/**< USART5 */
#define DEFAULT_UART "/dev/ttyS1";		/**< USART? */
#define HOTT_READ_TIMEOUT_MS	1000
//Delay before a HoTT answer. at least 5ms. The tweaked value for PX4 is around 4500
#define POST_READ_DELAY_IN_USECS	4500
//Delay between each HoTT msg bytes. Should be slightly above 2ms
#define POST_WRITE_DELAY_IN_USECS	2000

//function prototypes
extern "C" __EXPORT int ap_hott_main(int argc, char *argv[]);
int ap_hott_thread_main(int argc, char *argv[]);
int open_uart(const char *device);
int recv_req_id(int uart, uint8_t *mode, uint8_t *id);
int send_data(int uart, uint8_t *buffer, size_t size);
void initOrbSubs(void);

void hott_handle_text_mode(int uart, uint8_t moduleId);
void hott_handle_binary_mode(int uart, uint8_t moduleId);
void hott_send_vario_msgs(int uart);
void hott_send_eam_msg(int uart);
void hott_send_gps_msg(int uart);
bool checkTopic(int topic);

//variables
static int ap_hott_task;				/**< Handle of deamon task / thread */
static const char commandline_usage[] = "usage: ap_hott start|status|stop [-d <device>]\n      default device is ttyS1";
static int thread_should_exit = false;		/**< Deamon exit flag */
static int thread_running = false;		/**< Deamon status flag */

//ORB ids
static int _battery_sub = -1;
static int _sensor_sub = -1;
static int _apData_sub = -1;

struct battery_status_s battery;
struct sensor_combined_s sensors;
struct ap_data_s ap_data;

const char hott_flight_mode_strings[13+1][10] = {
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


int open_uart(const char *device)
{
	/* baud rate */
	static const speed_t speed = B19200;

	/* open uart */
	const int uart = open(device, O_RDWR | O_NOCTTY);

	if (uart < 0) {
		err(1, "Error opening port: %s", device);
	}
	
	/* Back up the original uart configuration to restore it after exit */	
	int termios_state;
	struct termios uart_config_original;
	if ((termios_state = tcgetattr(uart, &uart_config_original)) < 0) {
		close(uart);
		err(1, "Error getting baudrate / termios config for %s: %d", device, termios_state);
	}

	/* Fill the struct for the new configuration */
	struct termios uart_config;
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	warnx("uart_config.c_oflag = 0x%x", uart_config.c_oflag);
	uart_config.c_oflag &= ~OPOST;	//disable post processing
	warnx("uart_config.c_oflag = 0x%x", uart_config.c_oflag);
//	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		close(uart);
		err(1, "Error setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)",
			 device, termios_state);
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		close(uart);
		err(1, "Error setting baudrate / termios config for %s (tcsetattr)", device);
	}

	/* Activate single wire mode */
	ioctl(uart, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);
	tcflush(uart, TCIOFLUSH);	//flush input & output

	return uart;
}

/**
	receive requested HoTT mode and module id
**/
int recv_req_id(int uart, uint8_t *mode, uint8_t *id)
{
	struct pollfd fds;
	fds.fd = uart;
	fds.events = POLLIN;
	if (poll(&fds, 1, HOTT_READ_TIMEOUT_MS) > 0) {
		/* Get the mode: binary, text, etc  */
		read(uart, mode, sizeof(*mode));
		/* Read the device ID being polled */
		read(uart, id, sizeof(*id));
	} else {
		warnx("UART timeout on TX/RX port");
		return !OK;
	}
	return OK;
}

int send_data(int uart, uint8_t *buffer, size_t size)
{
	usleep(POST_READ_DELAY_IN_USECS);

	uint16_t checksum = 0;
	for (size_t i = 0; i < size; i++) {
		if (i == size - 1) {
			/* Set the checksum: the first uint8_t is taken as the checksum. */
			buffer[i] = checksum & 0xff;
		} else {
			checksum += buffer[i];
		}
		write(uart, &buffer[i], sizeof(buffer[i]));
		/* Sleep before sending the next byte. */
		usleep(POST_WRITE_DELAY_IN_USECS);
	}

	/* A hack the reads out what was written so the next read from the receiver doesn't get it. */
	/* TODO: Fix this!! */
	uint8_t dummy[size];
	read(uart, &dummy, size);
	return OK;
}

int ap_hott_main(int argc, char *argv[])
{
	if (argc < 1) {
		errx(1, "missing command\n%s", commandline_usage);
	}
	
	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("deamon already running");
			exit(0);
		}
		thread_should_exit = false;
		ap_hott_task = task_spawn_cmd("ap_hott",
		      SCHED_DEFAULT,
		      SCHED_PRIORITY_MAX - 5,
		      2048,
		      ap_hott_thread_main,
		      (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	} else if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	} else if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("daemon is running");

		} else {
			warnx("daemon not started");
		}

		exit(0);
	} else {
		errx(1, "unrecognized command\n%s", commandline_usage);
	}
	return OK;
}

void updateOrbs(void);

void initOrbSubs(void) {
	memset(&battery, 0, sizeof(battery));
	_battery_sub = orb_subscribe(ORB_ID(battery_status));

	memset(&sensors, 0, sizeof(sensors));
	_sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	
	memset(&ap_data, 0, sizeof(ap_data));
	_apData_sub = orb_subscribe(ORB_ID(ap_data));
}

int ap_hott_thread_main(int argc, char *argv[]) {
		thread_running = true;
		/* read commandline arguments */
		const char *device = DEFAULT_UART;
		for (int i = 0; i < argc && argv[i]; i++) {
			if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //device set
				if (argc > i + 1) {
					device = argv[i + 1];
				} else {
					thread_running = false;
					errx(1, "missing parameter to -d\n%s", commandline_usage);
				}
			}
		}

		if(device != NULL) {
			const int uart = open_uart(device);
			if (uart < 0) {
				errx(1, "Failed opening HoTT UART, exiting.");
				thread_running = false;
			} else {
				//init ORB subscriptions
				initOrbSubs();
				//main loop
				while(!thread_should_exit) {
					uint8_t mode = 0;
					uint8_t moduleId = 0;
					if(recv_req_id(uart, &mode, &moduleId) == OK) {
						updateOrbs();
						switch(mode) {
							case HOTT_TEXT_MODE_REQUEST_ID:
								hott_handle_text_mode(uart, moduleId);
								break;
							case HOTT_BINARY_MODE_REQUEST_ID:
								hott_handle_binary_mode(uart, moduleId);
								break;
							default:
								warnx("unknown HoTT request 0x%02x for id 0x%02x", mode, moduleId);
								break;
						}
					}
				}
				close(uart);
				warnx("Exiting...");
				thread_running = false;
			}
		}
	return 0;
}

void hott_handle_text_mode(int uart, uint8_t moduleId) {
	//TODO: reimplement
}

void hott_handle_binary_mode(int uart, uint8_t moduleId) {
	switch(moduleId) {
		case HOTT_TELEMETRY_GPS_SENSOR_ID:
//			warnx("GPS");
			hott_send_gps_msg(uart);
			break;
		case HOTT_TELEMETRY_EAM_SENSOR_ID:
//			warnx("EAM");
			hott_send_eam_msg(uart);
			break;
		case HOTT_TELEMETRY_VARIO_SENSOR_ID:
//			warnx("VARIO");
//			hott_send_vario_msgs(uart);
			break;
		case HOTT_TELEMETRY_GAM_SENSOR_ID:
//			warnx("GAM");
			break;
		case HOTT_TELEMETRY_AIRESC_SENSOR_ID:
//			warnx("AIRESC");
			break;
		case HOTT_TELEMETRY_NO_SENSOR_ID:
//			warnx("NO SENSOR?");
			break;
		default:
			warnx("binary mode request for unknown module id 0x%02x", moduleId);
			break;
	}
}

bool checkTopic(int topic) {
	bool updated;
	orb_check(topic, &updated);
	return updated;
}

void updateOrbs(void) {
	if(checkTopic(_battery_sub))
		orb_copy(ORB_ID(battery_status), _battery_sub, &battery);
	
	if(checkTopic(_sensor_sub))
		orb_copy(ORB_ID(sensor_combined), _sensor_sub, &sensors);

	if(checkTopic(_apData_sub))
		orb_copy(ORB_ID(ap_data), _apData_sub, &ap_data);
}

void hott_send_vario_msgs(int uart) {
	struct HOTT_VARIO_MSG msg;

	memset(&msg, 0, sizeof(struct HOTT_VARIO_MSG));
	msg.start_byte = 0x7c;
	msg.vario_sensor_id = HOTT_TELEMETRY_VARIO_SENSOR_ID;
	msg.sensor_id = 0x90;
	msg.stop_byte = 0x7d;
	sprintf((char *)msg.text_msg,"Hello!");
	send_data(uart, (uint8_t *)&msg, sizeof(struct HOTT_VARIO_MSG));
}


void hott_send_eam_msg(int uart) {
	struct HOTT_EAM_MSG msg;

	memset(&msg, 0, sizeof(struct HOTT_EAM_MSG));
	msg.start_byte = 0x7c;
	msg.eam_sensor_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
	msg.sensor_id = 0xe0;
	msg.stop_byte = 0x7d;
	//end init

	(uint16_t &)msg.main_voltage_L = (uint16_t)(battery.voltage_v * (float)10.0);
	(uint16_t &)msg.current_L = (uint16_t)(battery.current_a * (float)10.0);
	(uint16_t &)msg.batt_cap_L = (uint16_t)(battery.discharged_mah / (float)10.0);
	msg.temp1 = ap_data.temperature1 + 20;
	msg.temp2 = ap_data.temperature2 + 20;
	(int16_t &)msg.altitude_L = ap_data.altitude;
	(uint16_t &)msg.speed_L = ap_data.groundSpeed;
  	(uint16_t &)msg.climbrate_L = ap_data.climbrate;
  	msg.climbrate3s = 120 + (ap_data.climbrate / 100);  // 0 m/3s using filtered data here
  	
	//display ON when motors are armed
    if (ap_data.motor_armed) {
       msg.alarm_invers2 |= 0x80;
     } else {
       msg.alarm_invers2 &= 0x7f;
     }
	send_data(uart, (uint8_t *)&msg, sizeof(struct HOTT_EAM_MSG));
}


void hott_send_gps_msg(int uart) {
	struct HOTT_GPS_MSG msg;

	memset(&msg, 0, sizeof(struct HOTT_GPS_MSG));
	msg.start_byte = 0x7c;
	msg.gps_sensor_id = 0x8a;
	msg.sensor_id = 0xa0;
	msg.version = 0x00;
	msg.end_byte = 0x7d;

	(uint16_t &)msg.msl_altitude_L = ap_data.altitude / 100;  //meters above sea level  
  	msg.flight_direction = ap_data.groundCourse / 200; // in 2* steps
	(uint16_t &)msg.gps_speed_L = (uint16_t)ap_data.groundSpeed;
	
	 if(ap_data.gps_sat_fix == 3) {	//see GPS class
		msg.alarm_invers2 = 0;
		msg.gps_fix_char = '3';  
		msg.free_char3 = '3';  //3D Fix according to specs...
	  } else {
		//No GPS Fix
		msg.alarm_invers2 = 1;
		msg.gps_fix_char = '-';
		msg.free_char3 = '-';
		(uint16_t &)msg.home_distance_L = 0; // set distance to 0 since there is no GPS signal
	  }

  switch(ap_data.control_mode) {
  //TODO: switch to APM enums
	case 3:	//AUTO
	case 5:	//LOITER
     //Use home direction field to display direction an distance to next waypoint
		{
			(uint16_t &)msg.home_distance_L = ap_data.wp_distance / 100;
			msg.home_direction = ap_data.wp_direction / 200;
			//Display WP to mark the change of meaning!
			msg.free_char1 ='W';
			msg.free_char2 ='P';
		}
		break;

	default:
		//Display Home direction and distance
		{
			(uint16_t &)msg.home_distance_L = ap_data.home_distance / 100;
			msg.home_direction = ap_data.home_direction / 200;
			msg.free_char1 = 0;
			msg.free_char2 = 0;
        	break;
    	}
	}
	
	//TODO: fix this...
	//latitude
	{
		float coor = ap_data.latitude / (float)10000000.0;
		if(coor < (float)0.0)
			coor *= (float)-1.0;
		int lat = coor;  //degree
		coor -= lat;
		lat *= 100;
  
		coor *= 60;
		int tmp = coor;  //minutes
		lat += tmp;	
		//seconds
		coor -= tmp;
		coor *= (float)10000.0;
		int lat_sec = coor;
		//
		// Longitude
		coor = ap_data.longitude / (float)10000000.0;
		if(coor < (float)0.0)
			coor *= (float)-1.0;
		int lng = coor;
		coor -= lng;
		lng *= 100;
  
		coor *= 60;
		tmp = coor;  //minutes
		lng += tmp;
		//seconds
		coor -= tmp;
		coor *= (float)10000.0;
		int lng_sec = coor;
	
		if(ap_data.latitude > 0) {
			msg.pos_NS = 0;	//north
		} else {
			msg.pos_NS = 1; //south
		}
	
	  (uint16_t &)msg.pos_NS_dm_L = lat;
	  (uint16_t &)msg.pos_NS_sec_L = lat_sec;
  
	  if(ap_data.longitude >= 0) {
		 msg.pos_EW = 0; //east
	  } else {
		 msg.pos_EW = 1; //west
	  }
	  (uint16_t &)msg.pos_EW_dm_L = lng;
	  (uint16_t &)msg.pos_EW_sec_L = lng_sec;

	  (uint16_t &)msg.altitude_L = (ap_data.altitude/ 100)+500;  //meters above ground

	  (int16_t &)msg.climbrate_L = 30000 + ap_data.climbrate;  
	  msg.climbrate3s = 120;// + (ap_data.climb_rate / 100);  // 0 m/3s
  
	  msg.gps_satelites = ap_data.satelites;
  
	  msg.angle_roll = ap_data.angle_roll / 200;
	  msg.angle_nick = ap_data.angle_nick / 200;
  
	  msg.angle_compass = ap_data.angle_compas / 2;
	  //hott_gps_msg.flight_direction = hott_gps_msg.angle_compass;	//

	  uint32_t t = ap_data.utc_time;
	  msg.gps_time_h = t / 3600000;
	  t -= (msg.gps_time_h * 3600000);
  
	  msg.gps_time_m = t / 60000;
	  t -= msg.gps_time_m * 60000;
  
	  msg.gps_time_s = t / 1000;
	  msg.gps_time_sss = t - (msg.gps_time_s * 1000);
	}
	
	send_data(uart, (uint8_t *)&msg, sizeof(struct HOTT_GPS_MSG));
}
