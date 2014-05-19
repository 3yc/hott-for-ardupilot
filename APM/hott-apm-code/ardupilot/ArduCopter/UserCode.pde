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
  _hott_update_telemetry_data();
#endif
}
