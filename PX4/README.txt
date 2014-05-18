== HoTT V4 implementation for PX4 / Pixhawk
== (c) 2012-2014 by Adam Majerczyk < majerczyk.adam@gmail.com >

*** Note: the APM code works a bit different compared to the original PX4 code from ETHZ (https://pixhawk.org). To get telemetry data from APM you need to use the hott-for-ardupilot code, the original hott driver is working for ETHZ setups only! ***

== Suported platforms:
	* PX4 (http://pixhawk.org/modules/px4fmu)
	* Pixhawk (http://pixhawk.org)

== How to compile:
	- setup your development enviroment as described here: http://dev.ardupilot.com/wiki/building-the-code/
	  Linux / Mac setup is recommended. Windows setups are not testet yet.

	- Checkout the code and get all dependencies:
		git clone https://github.com/3yc/hott-for-ardupilot.git
		cd hott-for-ardupilot
		git submodule init
		git submodule update

	- Create a config file
		cd PX4
		make config

	- (optional) Edit the PX4/ardupilot/config.mk. Normal you need to change the communication port
	- To build a quad version of Arducopter:
		make quad

	- To build a hexa version of ArduCopter:
		make hexa

	- After a successfull build you will find the *.px4 firmware files for deployment in ardupilot/ArduCopter directory. You can use APM Planner to deploy the firmware.
	
== Hardware setup / How to connect a HoTT receiver to PX4/Pixhawk:
The HoTT code uses per default /dev/ttyS1 on PX4 and /dev/ttyS6 on Pixhawk. You can change the serial port by editing the PX4/hott-px4-code/ardupilot/ArduCoper/mk/PX4/ROMFS/init.d/rc.APM file. Simon Wilks made a good video how to connect to PX4 (https://www.youtube.com/watch?v=0KkZRDJnRtk). Take a look at the Pixhawk wiki also: http://pixhawk.org/peripherals/telemetry/hott

ttySx <-> UARTx mapping:
===========================
Device	PX4		Pixhawk
ttyS0	UART1	UART3 (TELEM2)
ttyS1	UART2	UART2 (TELEM1)
ttyS2	UART5	UART
ttyS3	UART6	UART
ttyS4	N/A		UART6
ttyS5	N/A		UART7
ttYS6	N/A		UART8
