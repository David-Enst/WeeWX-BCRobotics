# WeeWX-BCRobotics
A WeeWX driver for the BC Robotics and SparkFun weather station Raspberry Pi based hardware.

weewx-bcrobo

Driver to collect data from the "Spark Fun" SEN-08942 RoHS weather meters 
connected to the BC Robotics interface.

See:
https://www.sparkfun.com/products/8942

https://www.bc-robotics.com/tutorials/raspberry-pi-weather-station-part-1/

INTRODUCTION

The BC Robotics interface uses the Aidafruit BME280 sensor for humidity, pressure, 
and temperature along with a ADS1x15 ADC (analog to digital converter) to read the 
wind direction. The rain and wind speed are measured through the GPIO interface by 
counting "ticks" (rotations or measures of water). The driver includes a erroneous
rain tick filter, since this sensor can emit single undesired ticks.

SOFTWARE SETUP

Before installing the BC Robotics driver, setup the required software by running the 
following commands (in this order):

[Install the Adafruit Python GPIO Library]

  sudo apt-get update
  
  sudo apt-get install build-essential python-pip python-dev python-smbus git
  
  git clone https://github.com/adafruit/Adafruit_Python_GPIO.git
  
  cd Adafruit_Python_GPIO
  
  sudo python setup.py install
  

[Install the Adafruit BME280 Library]

  cd
  
  git clone https://github.com/adafruit/Adafruit_Python_BME280.git
  
  cd Adafruit_Python_BME280
  
  sudo python setup.py install
  

[Test the BME280 Sensor if desired]

  python Adafruit_BME280_Example.py
  

[Install the ADS1x15 Library, the ADS1015 Analog to Digital chip (wind direction)]

  cd
  
  git clone https://github.com/adafruit/Adafruit_Python_ADS1x15.git
  
  cd Adafruit_Python_ADS1x15
  
  sudo python setup.py install
  

[Setup the DS18B20 temperature sensor and install the software library]

  cd
  
  sudo modprobe w1-gpio
  
  sudo modprobe w1-therm
  
  cd /sys/bus/w1/devices
  
  ls
  
  cd
  
  sudo apt-get install python-w1thermsensor
  
  
The “ls” command will display the contents of the devices 'folder' in the window. The 
DS18B20 shows up as an address something like "28-0316853d8fff" – but each sensor has 
a unique ID.

TESTING THE HARDWARE

You can test the sensors by running the included test app: BCRobotics-test-app.py

Run the Python IDE:

  >>idle

Now open the "BCRobotics-test-app.py" test app from the IDE and hit 'F5' to run it. It will 
continuously print out the readings from the sensors, including the value read from the ADC 
for the wind direction. Use these values (and a test template for the wind direction) to 
double check the values used in the driver. Hit "<ctrl> c" to stop the program.


DRIVER INSTALLATION

1) install weewx (see the weewx user guide)

2) download the driver

wget -O BCRobotics.zip https://github.com/David-Enst/WeeWX-BCRobotics/archive/master.zip

3) install the driver

wee_extension --install BCRobotics.zip

4) configure the driver

wee_config --reconfigure

5) start weewx

sudo /etc/init.d/weewx start

 NOTE: We have moved to the 21st Century, so the database is in METRIC!
       Therefore, if you have a database created already, then see:

         http://weewx.com/docs/customizing.htm#Changing_the_unit_system

CREDITS

Implementation of this driver was much easier thanks to the work by others, 
including BC-Robotics:

Joerg Raedler
  https://bitbucket.org/jraedler/sunnywebbox

Aleksandar Tsankov
  https://github.com/fullergalway/weewx-airmar-150wx-driver

Matthew Wall    
  https://github.com/matthewwall/weewx-sdr
