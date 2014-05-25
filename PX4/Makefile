#ArduCopter + HoTT make file

.PHONY: bl bl-flash config quad v1-upload v2-upload px4-hott clean

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
quad: px4-hott
	cd ardupilot/ArduCopter; $(MAKE) px4-quad

hexa: px4-hott
	cd ardupilot/ArduCopter; $(MAKE) px4-hexa

tri: px4-hott
	cd ardupilot/ArduCopter; $(MAKE) px4-tri

octa: px4-hott
	cd ardupilot/ArduCopter; $(MAKE) px4-octa

y6: px4-hott
	cd ardupilot/ArduCopter; $(MAKE) px4-y6

#upload ArduCopter to PX4
v1-upload:
	cd ardupilot/ArduCopter; $(MAKE) px4-v1-upload

#upload ArduCopter to Pixhawk
v2-upload:
	cd ardupilot/ArduCopter; $(MAKE) px4-v2-upload

px4-hott:
	cp -Rv hott-px4-code/ardupilot/* ardupilot
	cp -Rv hott-px4-code/PX4Firmware/* PX4Firmware

dist-hexa: hexa
	mkdir -p dist
	zip -j dist/AC-3.1.x-Hexa.zip PX4Firmware/Images/px4fmu-v1_APM.px4 PX4Firmware/Images/px4fmu-v2_APM.px4

dist-quad: quad
	mkdir -p dist
	zip -j dist/AC-3.1.x-Quad.zip PX4Firmware/Images/px4fmu-v1_APM.px4 PX4Firmware/Images/px4fmu-v2_APM.px4

dist-tri: tri
	mkdir -p dist
	zip -j dist/AC-3.1.x-Tri.zip PX4Firmware/Images/px4fmu-v1_APM.px4 PX4Firmware/Images/px4fmu-v2_APM.px4

dist-octa: octa
	mkdir -p dist
	zip -j dist/AC-3.1.x-Octa.zip PX4Firmware/Images/px4fmu-v1_APM.px4 PX4Firmware/Images/px4fmu-v2_APM.px4

dist-y6: y6
	mkdir -p dist
	zip -j dist/AC-3.1.x-Y6.zip PX4Firmware/Images/px4fmu-v1_APM.px4 PX4Firmware/Images/px4fmu-v2_APM.px4

dist: dist-quad dist-hexa dist-tri dist-octa dist-y6

clean:
	cd ardupilot/ArduCopter; $(MAKE) px4-clean
	cd PX4Firmware; $(MAKE) clean 
	cd PX4NuttX/nuttx; $(MAKE) clean
	rm dist/*
