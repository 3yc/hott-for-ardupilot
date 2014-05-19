Code for APM Atmel versions (APM1, APM2, APM2.x)

Cloning:
1.) git clone https://github.com/3yc/hott-for-ardupilot.git
2.) cd hott-for-ardupilot
3.) git submodule init
4.) git submodule update
5.) git checkout APM2-v2.9-HoTT
6.) git submodule init
7.) git submodule update

Compile:
1. cd APM
2. make config
3. edit ardpilot/config.mk. Add ARDUINO & ARDUINOS pointig to your Arduino 1.0.5 installation
4. make quad or make tri or make hex or... edit ardupilot/ArduCopter/APM_Config.h and run make
