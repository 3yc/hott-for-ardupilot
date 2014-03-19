#ArduCopter + HoTT make file

.PHONY: bl bl-flash config quad v1-upload v2-upload

#make bootloader
bl:
	$(MAKE) -C libopencm3 lib 
	$(MAKE) -C Bootloader

# flash bootloader
bl-flash: bl
	$(MAKE) -C Bootloader -f Makefile.f4 TARGET=fmu flash-bootloader

#Create ArduCopter config
config:
	cd ardupilot/ArduCopter; $(MAKE) configure
	echo -— please edit ardupilot/config.mk to match your needs —-

# build quad version
quad:
	cd ardupilot/ArduCopter; $(MAKE) px4-quad

#upload ArduCopter to PX4
v1-upload:
	cd ardupilot/ArduCopter; $(MAKE) px4-v1-upload

#upload ArduCopter to Pixhawk
v2-upload:
	cd ardupilot/ArduCopter; $(MAKE) px4-v2-upload
