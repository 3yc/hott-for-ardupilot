/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//HoTT for PX4 only
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_combined.h>

//AP own ORB object...
#include <uORB/topics/ap_data.h>
ORB_DEFINE(ap_data, struct ap_data_s);

//ORB handles
orb_advert_t hBatteryTopic;
orb_advert_t hSensorsTopic;
orb_advert_t hApDataTopic;
#endif

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
	//setup ORB publishing
	struct battery_status_s bs;
	memset(&bs, 0, sizeof(bs));
	hBatteryTopic = orb_advertise(ORB_ID(battery_status), &bs);

	struct sensor_combined_s sen;
	memset(&sen,0,sizeof(sen));
	hSensorsTopic = orb_advertise(ORB_ID(sensor_combined), &sen);
	
	struct ap_data_s apData;
	memset(&apData, 0, sizeof(apData));
	hApDataTopic = orb_advertise(ORB_ID(ap_data), &apData);

	//setup ORB publishing
#endif
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
	struct battery_status_s bs;
	bs.timestamp = hal.scheduler->micros();
	bs.voltage_v = battery.voltage();
	bs.current_a = battery.current_amps();
	bs.discharged_mah = battery.current_total_mah();
	
	orb_publish(ORB_ID(battery_status), hBatteryTopic, &bs);
	
	struct sensor_combined_s sen;
	memset(&sen,0,sizeof(sen));
	sen.timestamp = hal.scheduler->micros();
	sen.baro_pres_mbar = barometer.get_pressure() / 100;
	sen.baro_alt_meter = baro_alt;
	sen.baro_temp_celcius = barometer.get_temperature();
	sen.mcu_temp_celcius = 0;
	orb_publish(ORB_ID(sensor_combined), hSensorsTopic, &sen);
	
	struct ap_data_s apData;
	memset(&apData, 0, sizeof(apData));
	apData.timestamp = hal.scheduler->micros();
	apData.battery1_v = 0;
	apData.battery2_v = 0;
	apData.temperatur1 = barometer.get_temperature();
	apData.temperatur2 = 0;
	apData.altitude = g_gps->altitude_cm;
	apData.groundSpeed = ((float)((g_gps->ground_speed_cm / 100) * 0.036));
	apData.groundCourse = g_gps->ground_course_cd;
	apData.climbrate = 30000 + climb_rate;
	
	apData.motor_armed = motors.armed();
	apData.control_mode = control_mode;
	
	apData.home_distance = home_distance; //in cm
	apData.home_direction = home_bearing; //in centi-degrees

	apData.wp_distance = wp_distance; //in cm
	apData.wp_direction = wp_bearing; //in centi-degrees

	apData.latitude = g_gps->latitude;
	apData.longitude = g_gps->longitude;
	apData.satelites = g_gps->num_sats;
	apData.gps_sat_fix = g_gps->status();
	apData.angle_roll = ahrs.roll_sensor;
	apData.angle_nick = ahrs.pitch_sensor;
	apData.angle_compas = ToDeg(compass.calculate_heading(ahrs.get_dcm_matrix())) ;
	apData.utc_time = g_gps->time_week_ms % (60 * 60 * 24 * 7);

	orb_publish(ORB_ID(ap_data), hApDataTopic, &apData);
	
#endif
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif