# Copyright 2024 David W. Enstrom, all rights reserved
# Distributed under the terms of the GNU Public License (GPLv3)
"""
Driver to collect data from the "Spark Fun" SEN-08942 RoHS
weather meters and the BC Robotics interface.

- Updated to use Python V3
- Finer grained error reporting

See:    https://www.sparkfun.com/products/8942 
        https://www.bc-robotics.com/tutorials/raspberry-pi-weather-station-part-1/


NOTE: With the Bookworms version of Raspbian Python is installed in a virtual environment.
      The following reference may be required for the driver to work properly with this environment. 
      Change the references 'USER' and 'VIRTUALENV' as required for your install.

To make testing easier:
import sys	# Define path to virtual PYTHON libraries
sys.path.extend(["/home/USER/VIRTUALENV/lib64/python3.11/site-packages"])


"""

import time
import datetime
import board
import busio
import syslog
import threading
import math

from adafruit_bme280 import basic as adafruit_bme280
from w1thermsensor import W1ThermSensor
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Use the button object to detect the wind speed and rain
from gpiozero import Button

# Set  PATH for files (pin factory issue w driver as daemon )
from os import chdir

import weewx.drivers
import weewx.units
import weewx.accum

DRIVER_NAME = 'BCRobotics'
DRIVER_VERSION = '3.3.12'

def logmsg(dst, msg):
    syslog.syslog(dst, 'BCRobo: %s' % msg)
    
def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)

windTick = 0     # Count of the wind speed input trigger
rainTick = 0     # Count of the rain input trigger
interval = 3     # Set now to define scope of variable
rainTime = 0     # Use this to detect erroneous rain events
last_out = 0     # To handle temp sensor errors

# Define now to set wide scope
out_Temp = 0     # Set now to define scope for rain check
windSpeed = 0    # km/h
windDir = 0      # compass deg
out_Temp = 0     # degree_C
rain = 0         # cm as per default
rainRate = 0     # cm/hr
pressure = 0     # mbar from the BME280
case_temp = 0    # degree_C
in_humidity = 0  # percent
out_humidity = 0 # percent
DewPoint = 0     # degree_C
wind_chill = 0   # degree_C
heat_index = 0   # degree_C
altitude = 0     # altitude from the BME280
val = 0          # wind direction value


TempSensor = False  # Temperature sensor good flag
I2CSensor =  False  # I2C interface flag (BME280 & Wind Dir)
WindSensor = False  # Wind direction sensor good flag
WindSpSens = False  # Wind speed sensor good flag
RainSensor = False  # Rain measurement sensor good flag

loginf('Driver version - %s' % DRIVER_VERSION)

# Setup Temperature sensor
try:
    ds18b20 = W1ThermSensor()
    time.sleep(interval)

except Exception as err:
    logerr('Temperature Sensor Error: %s' % err)
    TempSensor = False
else:
    loginf('W1ThermSensor setup fine.')
    TempSensor = True

# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)

# Setup wind direction ADC
try:
     ads = ADS.ADS1015(i2c)
     ads.gain = 1
     chan = AnalogIn(ads, ADS.P0)
except Exception as err:
     logerr('Wind Direction setup Error: %s' % err)
     I2CSensor = False
else:
    loginf('Wind Direction setup fine.')
    I2CSensor = True

# Setup case temp, pressure, & humidity
try:
     bme = adafruit_bme280.Adafruit_BME280_I2C(i2c) 

except Exception as err:
     logerr('BME280 Error: %s' % err)
     I2CSensor = False
else:
     I2CSensor = True
     loginf('BME280 setup fine.')
     
def loader(config_dict, _):
    return BCRoboDriver(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return BCRoboConfEditor()

class BCRoboConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """

[BCRobotics]
    # This section is for the "Spark Fun" SEN-08942 / BC Robotics weather stations.
    # See: https://www.sparkfun.com/products/8942
    #      https://www.bc-robotics.com/tutorials/raspberry-pi-weather-station-part-1/

    # The time (in seconds) between LOOP packets. Default is:
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
    
"""

# flags for enabling/disabling debug verbosity
DEBUG_COMM = 0
DEBUG_CONFIG_DATA = 0
DEBUG_WEATHER_DATA = 0
DEBUG_HISTORY_DATA = 0
DEBUG_DUMP_FORMAT = 'auto'



class BCRoboDriver(weewx.drivers.AbstractDevice):
    """weewx driver that communicates with a "Spark Fun" SEN-08942 / BC Robotics station

    mode - Communication mode - TCP, UDP, or Serial.
    [Required. Default is serial] 
    
    port - Serial port or network address.
    [Required. Default is /dev/ttyS0 for serial,
     and 192.168.36.25:3000 for TCP/IP]

    loop_interval - The time (in seconds) between LOOP packets.
    [Required. Default is loop_interval = 3]
    
    timeout - The amount of time, in seconds, before the connection fails if
    there is no response.
    [Optional. Default is 3]

    debug_read - The level of message logging. The higher this number, the more
    information is logged.
    [Optional. Default is 0]
    
    """
    def __init__(self, **stn_dict):
        global rainTime
        global interval
        global RainSensor
        rainTime = int(time.time())   # used to detect erronious rain ticks
        interval = int(stn_dict.get('loop_interval', 3))
        
        # Set  PATH for file creation (pin factory issue w driver as daemon )
        chdir('/tmp') 
        
        # Define event to detect wind (4 ticks per revolution)
        #  1 tick/sec = 1.492 mph or 2.4 km/h
        #  So: wind = (windTick * 2.4) / loop_interval
        def windtrig(self):
            global windTick
            windTick += 1
            #logerr('Wind Speed tick: ' + str(windTick))


        # Set digital pin 17 to an input (wind speed)
        global WindSpSens
        try:
            windEvent = Button(17)

        except Exception as err:
            logerr('Wind Speed Error: %s' % err)
            WindSpSens = False
        else: 
            WindSpSens = True
            loginf('Wind Speed setup fine.')      

        # Define event callbacks for both 'pressed' and 'released'
        if WindSpSens : 
            windEvent.when_pressed = windtrig
            windEvent.when_released = windtrig

        # Define event to detect rain (0.2794mm per tick)
        # with a 1hr time period where the first tick is ignored
        # which is used to detect an improper single tick
        def raintrig(self):
            global rainTick
            rainTick += 1
            #logerr('Rain ticked: ' + str(rainTick))
            #print ("Rain ticked: " + str(rainTick))

        # Set digital pin 23 to an input (rain)
        try:
            rainEvent = Button(23)

        except Exception as err:
            logerr('Rain Error: %s' % err)
            RainSensor = False
        else: 
            RainSensor = True
            loginf('Rain setup fine.')
        
        # Define event callbacks for 'pressed'
        if RainSensor :
            rainEvent.when_pressed = raintrig
        
    @property
    def hardware_name(self):
        return "BCRobotics"
    
    def genLoopPackets(self):
        while True:
            packet = {'dateTime': int(time.time() + 0.0),
                      'usUnits': weewx.METRIC}
            readings = StationData.get_readings()
            packet.update(readings)
    #        self._augment_packet(packet)
            yield packet

    #def _augment_packet(self, packet):
    #    # calculate the rain delta from rain total
    #    if self.last_rain is not None:
    #        packet['rain'] = packet['rain_total'] - self.last_rain
    #    else:
    #        packet['rain'] = None
    #    self.last_rain = packet['rain_total']

# ======================================================================= #
#       StationData class - setup stn and get data from the device        #
# ======================================================================= #


class StationData():
    def __init__(self):
       # Initialize the setup and define ports
       last_rain = None
       global windTick
       global rainTick
       windTick = 0   # Count of the wind speed input trigger
       rainTick = 0   # Count of the rain input trigger


    @staticmethod
    def get_readings():
        """ The "Spark Fun" SEN-08942 / BC Robotics station emits data
         through a BME280 interface, a ds18b20 temperature sensor plus
         a ADS1015 Analog to Digital chip for the wind direction.

        https://www.bc-robotics.com/tutorials/raspberry-pi-weather-station-part-2/

        The following data is provided by the sensor hardware:

          windSpeed    - from total ticks (1 tick/sec = 2.4 km/h)
          windDir      - wind direction (0-255)
          out_Temp     - outdoor temperature (in Deg C)
          rain         - rain during the loop_interval (1 tick = 0.2794 mm)
          pressure     - pressure (in Pascals)
          case_temp    - hardware case temperature (in Deg C)
          out_humidity - outdoor humidity (calculated, see below)
          in_humidity  - humidity inside the case (+/-0.1 %)
          
        """
        #
        # Get the readings, define the data list, and fill it in from readings
        # NOTE: The pressure reading is 'pressure', not 'barometer'
        #

        # Declair all required globals (stupid Python)
        global case_temp
        global pressure
        global in_humidity
        global altitude
        global I2CSensor
        global out_Temp
        global last_out
        global TempSensor
        global DewPoint
        global out_humidity
        global val
        global WindSensor
        global windDir 
        global windTick
        global windSpeed
        global WindSpSens
        global rainTick
        global rain
        global RainSensor
        global rainRate
        global wind_chill
        global heat_index
                                                
        # If sensor OK, get case Temperature from BME280 in degrees_C
        if I2CSensor :
            case_temp = bme.temperature      
            # Get Barometric Pressure from BME280 in mbar
            #   mbar = hPa
            pressure = bme.pressure  
            #loginf('BCRobo Pressure: ' + str(pressure))
            # Get Humidity (inside the case) from BME280 in %
            in_humidity = bme.humidity
            #loginf('BCRobo Humidity: ' + str(in_humidity))
            altitude = bme.altitude
        else: 
            case_temp = 20
            pressure = 990
            in_humidity = 90

        # Get temperature from DS18B20 sensor in degrees_C
        out_temp = 0
        if TempSensor : # If Sensor connected
            try:
                out_Temp = ds18b20.get_temperature()
                last_out = out_Temp
            except Exception as err:
                logerr('Temp Sensor Error: %s' % err)
                out_Temp = last_out
        else: 
            out_Temp = case_temp
        
        # This humidity is measured inside the case, which is warmer than the 
        # ambient air. Therefore it is converted to external humidity based
        # upon the case_temp, in_humidity, pressure, and out_Temp. 
        # 
        # Using the NOAA formulae:
        VapPress = (6.112 * math.exp(17.67 * case_temp / (case_temp + 243.5))) * (in_humidity/100)
        if VapPress <= 0:
            logerr('VapPress error: ' + str(VapPress))
            VapPress = 3
        DewPoint = (243.5 * math.log(VapPress / 6.112))/(17.67 - math.log(VapPress / 6.112))
        absVapPress = 6.11 * math.pow(10, (7.5 * DewPoint / (237.7 + DewPoint)))
        if absVapPress == pressure:
            absVapPress = absVapPress + 1
            logerr('absVapPress error: ' + str(pressure))
        actMixRatio = 621.97 * absVapPress / (pressure - absVapPress)
        
        # Adjust humidity reading to the outside temperature
        # using the NOAA formulae:
        out_humidity = actMixRatio * 10 / (0.42 * math.exp(out_Temp * 0.06235394))
        if out_humidity > 100:
            out_humidity = 100.0

        # Calculate wind direction (angle) based on ADC reading
        #   Read ADC channel 0 with a gain of 1
        if I2CSensor :
            val = chan.value
        else:
            val = 7000
           
        windDir = 1.5 # Dummy test value

        if 19600 <= val <= 20999:
            windDir = 0

        if 9000 <= val <= 10799:
            windDir = 22.5

        if 10800 <= val <= 13999:
            windDir = 45

        if 2000 <= val <= 2299:
            windDir = 67.5

        if 2300 <= val <= 2999:
            windDir = 90

        if 1000 <= val <= 1999:
            windDir = 112.5

        if 4000 <= val <= 4999:
            windDir = 135

        if 3000 <= val <= 3999:
            windDir = 157.5

        if 6600 <= val <= 8999:
            windDir = 180

        if 5000 <= val <= 6599:
            windDir = 202.5

        if 15900 <= val <= 17499:
            windDir = 225

        if 14000 <= val <= 15899:
            windDir = 247.5

        if 24000 <= val <= 24999:
            windDir = 270

        if 21000 <= val <= 21999:
            windDir = 292.5

        if 22000 <= val <= 23999:
            windDir = 315

        if 17500 <= val <= 19599:
            windDir = 337.5

        if 26000 <= val <= 26500: # The value when no sensor is attached
            windDir = 180

        if windDir == 1.5:
            logerr('Wind direction error: ' + str(val))

        # Calculate the average wind speed over this 'interval' (km/h)
        #  1 tick/sec = 1.492 mph or 2.4 km/h
        #  So wind speed = (windTick * 2.4) / loop_interval
        if WindSpSens :
            windSpeed = (windTick * 2.4) / interval
            windTick = 0
        else:
            windSpeed = 0
            windTick = 0
            
        # Calculate the rainfall over this 'interval' (cm)
        # 1 tick = 0.011 inches or 0.02794 cm 
        # So rain = rainTick * 0.02794
        if RainSensor: rain = rainTick * 0.02794
        else: rain = 0
        #logerr('Rain ticks: ' + str(rainTick) + " Rain: " + str(rain))
        
        # Calculate Windchill 
        # Using the NOAA formulae:
        wind_chill = out_Temp
        if out_Temp < 0 and windSpeed > 0 and windSpeed < 100: 
            if windSpeed >= 5:
                wind_chill = 13.12 + (0.6215 * out_Temp) - (11.37 * pow(windSpeed, 0.16)) + (0.3965 * out_Temp * pow(windSpeed, 0.16))
            elif windSpeed < 5:
                wind_chill = float(out_Temp) + ((((-1.59) + (0.1345*out_Temp))/5) * windSpeed)

        # Calculate Outside Dew Point
        # Using the NOAA formulae:
        VapPress = (6.112 * math.exp(17.67 * out_Temp / (out_Temp + 243.5))) * (out_humidity/100)
        if VapPress <= 0:
            loginf('VapPress error: ' + str(VapPress))
            VapPress = 3
        DewPoint = (243.5 * math.log(VapPress / 6.112))/(17.67 - math.log(VapPress / 6.112))

        # Calculate Humidex 
        # Using the NOAA formulae:
        heat_index = out_Temp
        if  out_Temp > 19:
            heat_index = float(out_Temp) + 0.5555 * (6.11 * math.exp(5417.753 * ((1/273.16) - (1/(273.15 + float(DewPoint))))) - 10)

        if rain > 0:
        #    Calculate the rain rate based up the rain this interval
        #    if the rain is non-zero
            rainRate = (rain / interval) * 3600/interval  # cm/h
            rainTick = 0
        else:
            rainTick = 0
            rainRate = None
            rain = None 
            
        data = dict()
        data['windSpeed'] = windSpeed       # km/h
        data['windDir'] = windDir           # compass deg
        data['outTemp'] = out_Temp          # degree_C
        data['rain'] = rain                 # cm as per default
        data['rainRate'] = rainRate         # cm/hr
        data['pressure'] = pressure         # mbar
        data['inTemp'] = case_temp          # degree_C
        data['inHumidity'] = in_humidity    # percent
        data['outHumidity'] = out_humidity  # percent
        data['dewpoint'] = DewPoint         # degree_C
        data['windchill'] = wind_chill      # degree_C
        data['heatindex'] = heat_index      # degree_C
        
        rain = 0
        rainRate = 0
        return data

    def __exit__(self, _, value, traceback):
        windEvent.close()
        rainEvent.close()
        #self.close()
        return 

    def __enter__(self):
        #self.open()
        return self


# Define a main entry point for basic testing of the station without weewx
# engine and service overhead.  Invoke this as follows from the WeeWX root 
# dir (i.e., /usr/bin/weewxd):
#
# sudo PYTHONPATH=/usr/share/weewx python /usr/share/weewx/user/BCRobotics.py
# 

if __name__ == '__main__':

    syslog.openlog('BCRobotics', syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))
    
    with StationData() as s:
        # Print results
        while True:
            s.get_readings()
            ts = time.time()
            myTime = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            print("\nTime: " , myTime)
            print("Temperature: %0.1f C" % out_Temp)
            print("Case Temperature: %0.1f C" % case_temp)
            print("Humidity: %0.1f %%" % in_humidity)
            print("Out Humidity: %0.1f %%" % out_humidity)
            print("Dewpoint: %0.1f C" % DewPoint)
            print("Pressure: %0.2f hPa" % pressure)
            print("Altitude = %0.2f meters" % altitude)
            print( 'Wind Dir:    ' , windDir, ' (', val, ')')
            print ("Wind Speed: %0.2f km/h" % windSpeed)
            print("Wind Chill: %0.1f C" % wind_chill)
            print("Heat Index: %0.1f C" % heat_index)
            print ("Rainfall:   %0.2f  mm" % rain)
            print ("Rainrate:   %0.2f  mm" % rainRate)
            print (" ")
            time.sleep(interval)
