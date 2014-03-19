/**
 * @file ap_data.h
 *
 * Definition of the ArduPilot status uORB topic.
 */

#ifndef AP_DATA_H_
#define AP_DATA_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * 
 */

struct ap_data_s {
	uint64_t	timestamp;		/**< microseconds since system boot, needed to integrate */

	uint16_t	battery1_v;	//Battery 1 voltage
	uint16_t	battery2_v;	//Battery 2 voltage
	
	float		temperatur1;	//scale °C
	float		temperatur2;	//scale °C
	uint16_t	altitude;		// in cm
	
	uint16_t	groundSpeed;	//	ground speed in km/h
	uint16_t	groundCourse;	// ground course in 100ths of a degree
	uint16_t 	climbrate;		//// The cm/s we are moving up or down based on filtered data - Positive = UP
	
	bool	motor_armed;	//motors armed?
	int8_t	control_mode;	//Stabilise, loiter, auto etc.

	uint16_t	home_distance;	//meters
	uint16_t	home_direction;	//direction from starting point to Model position

	uint16_t	wp_distance;	//waypoint distance in meters
	uint16_t	wp_direction;	//waypoint bearing
	
	int32_t latitude;	//latitude in degrees * 10,000,000
	int32_t longitude;	//longitude in degrees * 10,000,000
	
	uint8_t	satelites;	//GPS visible sat
	uint8_t gps_sat_fix;
	
	int8_t	angle_roll;
	int8_t	angle_nick;
	int8_t	angle_compas;
	
	uint32_t	utc_time;	// UTC in ms
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(ap_data);
#endif