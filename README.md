# WeeWX-BCRobotics V3
A Python V3 WeeWX driver for the BC Robotics and SparkFun weather station Raspberry Pi based hardware.

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

If you want to build your own system see the documentation provided in the attached "Issue".

SOFTWARE SETUP

Before installing the BC Robotics driver, setup the required software by running the 
following commands (in this order):

[Install the Adafruit Python GPIO Library]

sudo apt-get update
sudo pip3 install --upgrade setuptools
pip3 install RPI.GPIO
pip3 install adafruit-blinka
  

[Install the Adafruit BME280 Library]

sudo pip3 install adafruit-circuitpython-bme280

[Install the ADS1x15 Library, the ADS1015 Analog to Digital chip (wind direction)]

sudo pip3 install adafruit-circuitpython-ads1x15  

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

  >>thonny

Now open the "BCRobotics-test-app.py" test app from the IDE and hit 'F5' to run it. It will 
continuously print out the readings from the sensors, including the value read from the ADC 
for the wind direction. Use these values (and a test template for the wind direction) to 
double check the values used in the driver. Hit "<ctrl> c" to stop the program.


DRIVER INSTALLATION

1) install weewx (see the weewx user guide)

2) download the driver and the documentation:

wget -O BCRobotics.zip https://github.com/David-Enst/WeeWX-BCRobotics/archive/master.zip

3) Extract the BCRobotics.py driver file and copy it to the Pi here:

usr/share/weewx/user/BCRobotics.py

4) Edit the etc/weewx/weewx.config file as follows:

Change the station type:
station_type = BCRobotics

Add the BCRobotics driver definition:
[BCRobotics]
    # This defines the "Spark Fun" SEN-08942 / BC Robotics weather stations.
    # See:    https://www.sparkfun.com/products/8942
    #         https://www.bc-robotics.com/tutorials/raspberry-pi-weather-station-part-1/
    
    # The time (in seconds) between LOOP packets.
    loop_interval = 3
    
    # Driver mode - tcp, udp, or serial
    mode = serial
    
    # If serial, specify the serial port device. (ex. /dev/ttyS0, /dev/ttyUSB0,
    # or /dev/cuaU0)
    # If TCP, specify the IP address and port number. (ex. 192.168.36.25:3000)
    port = /dev/ttyS0
    
    # The amount of time, in seconds, before the connection fails if there is
    # no response
    timeout = 3
    
    # Debug level - the level of message logging. The higher
    # the number, the more info is logged.
    debug_read = 0
    
    # The driver to use:
    driver = user.BCRobotics
    
Verify the definition of [[[Units]]] as follows:
        [[[Units]]]
            [[[[Groups]]]]
                group_altitude    = meter
                group_pressure    = hPa
                group_rain        = mm
                group_rainrate    = mm_per_hour
                group_temperature = degree_C
                group_degree_day  = degree_C_day
                group_speed       = km_per_hour
                group_speed2      = km_per_hour2

Verify the definition of [[[Labels]]] to ensure the web pages generated display the correct labels:
        [[[Labels]]]
            # Generic labels, keyed by an observation type.
            [[[[Generic]]]]
                barometer      = Barometer
                dewpoint       = Dew Point
                ET             = ET
                heatindex      = Heat Index
                inHumidity     = Case Humidity
                inTemp         = Case Temperature
                
Update the [StdConvert] section is to reflect the fact that all data in the database is stored in METRIC by default:
    target_unit = METRIC    # Options are 'US', 'METRICWX', or 'METRIC'
    
The database and driver is setup to store values in METRIC by default. Therefore, the following updates to the quality control parameters are required:
[StdQC]
    [[MinMax]]
        barometer =   800, 1110, mbar # = hPa
        pressure =    800, 1110, mbar  # = hPa
        outTemp =     -50, 49, degree_C
        inTemp =      -25, 49, degree_C
        outHumidity = 0, 100
        inHumidity =  0, 100
        windSpeed =   0, 300, km_per_hour
        rain =        0, 250, mm

Finally, update the archive section to force WeeWX to generate archive records through software generation, since the BCRobotics hardware does not generate them:
[StdArchive]
    # If possible, new archive records are downloaded from the station
    # hardware. If the hardware does not support this, then new archive
    # records will be generated in software.
    # Set the following to "software" to force software record generation.
    record_generation = software
   
    
5) Restart WeeWX:

sudo /etc/init.d/weewx stop
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
