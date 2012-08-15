// agmatthews USERHOOKS

void userhook_init()
{
	// put your initialisation code here
#ifdef HOTT_TELEMETRY
	_hott_setup();
#endif
}

void userhook_50Hz()
{
  // put your 50Hz code here
#ifdef PPM_WATCHDOG
//PPM watchdog
	static uint16_t old_icr = 0;
	static uint8_t icr_same_count = 0;
	if(motors.armed()) {	//we are up in the sky
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
		uint16_t icr = ICR5;
#endif
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM1
		uint16_t icr = ICR4;
#endif
		if(icr != old_icr) {
			old_icr = icr; 	//everything ok, ICR5 changed
			if(icr_same_count != 0)
				icr_same_count = 0;
			if(failsafe) {
				//clean fail safe
				set_failsafe(false);
			}
		} else {
			//same value?
			if(icr_same_count > PPM_WATCHDOG_TRIGGER * 50) {
				//about 3sec without a change, DO something!
				//Serial.print("\n!!! SIGNAL LOST !!!\n");
				if(!failsafe)
					set_failsafe(true);
			} else {
				icr_same_count++;
			}
		}
	}
//
#endif

#ifdef HOTT_TELEMETRY
#ifdef HOTT_ALARMS
	_hoot_check_alarm();
	_hott_alarm_scheduler();
	_hott_update_replay_queue();
#endif
#endif

}
