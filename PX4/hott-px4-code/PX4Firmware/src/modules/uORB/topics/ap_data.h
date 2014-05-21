/**
 * @file ap_data.h
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
// Check project homepage at https://github.com/3yc/hott-for-ardupilot
//
// 01/2013 by Adam Majerczyk (majerczyk.adam@gmail.com)
//
//

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
 * ArduPilot Data topics
 */

struct ap_data_s {
	uint64_t	timestamp;		/**< microseconds since system boot, needed to integrate */

	uint16_t	battery1_v;	//Battery 1 voltage
	uint16_t	battery2_v;	//Battery 2 voltage
	
	float		temperature1;	//scale °C
	float		temperature2;	//scale °C
	int16_t		altitude;		// in cm
	int16_t		altitude_rel;	// relative altitude to home location
	
	uint16_t	groundSpeed;	//	ground speed in km/h
	uint16_t	groundCourse;	// ground course in 100ths of a degree
	int16_t 	climbrate;		//// The cm/s we are moving up or down based on filtered data - Positive = UP
	
	bool		motor_armed;	//motors armed?
	uint8_t		control_mode;	//Stabilise, loiter, auto etc.

	uint16_t	home_distance;	//meters
	uint16_t	home_direction;	//direction from starting point to Model position

	uint16_t	wp_distance;	//waypoint distance in meters
	uint16_t	wp_direction;	//waypoint bearing
	
	int32_t 	latitude;	//latitude in degrees * 10,000,000
	int32_t 	longitude;	//longitude in degrees * 10,000,000
	
	uint8_t		satelites;	//GPS visible sat
	uint8_t 	gps_sat_fix;
	
	uint8_t		angle_roll;
	uint8_t		angle_nick;
	uint8_t		angle_compas;
	
	uint32_t	utc_time;	// UTC in ms
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(ap_data);
#endif