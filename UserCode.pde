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
#ifdef HOTT_TELEMETRY
#ifdef HOTT_ALARMS
	_hoot_check_alarm();
	_hott_alarm_scheduler();
	_hott_update_replay_queue();
#endif
#endif

}
