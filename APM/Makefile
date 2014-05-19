#ArduCopter + HoTT make file

.PHONY: bl bl-flash config quad v1-upload v2-upload px4-hott clean

#Create ArduCopter config
all:

config:
	cd ardupilot/ArduCopter; $(MAKE) configure
	echo -— please edit ardupilot/config.mk to match your needs —-

hott-patch:
	cp -Rf hott-apm-code/ardupilot/* ardupilot

# build quad version
quad: hott-patch
	echo "#define FRAME_CONFIG QUAD_FRAME" >> ardupilot/ArduCopter/APM_Config.h
	echo "#define FRAME_ORIENTATION X_FRAME" >> ardupilot/ArduCopter/APM_Config.h
	cd ardupilot/ArduCopter; $(MAKE) 

hexa: hott-patch
	echo "#define FRAME_CONFIG HEXA_FRAME" >> ardupilot/ArduCopter/APM_Config.h
	echo "#define FRAME_ORIENTATION X_FRAME" >> ardupilot/ArduCopter/APM_Config.h
	cd ardupilot/ArduCopter; $(MAKE) 

tri: hott-patch
	echo "#define FRAME_CONFIG TRI_FRAME" >> ardupilot/ArduCopter/APM_Config.h
	echo "#define FRAME_ORIENTATION X_FRAME" >> ardupilot/ArduCopter/APM_Config.h
	cd ardupilot/ArduCopter; $(MAKE) 

upload:
	cd ardupilot/ArduCopter; $(MAKE) upload

clean:
	cd ardupilot/ArduCopter; $(MAKE) clean

