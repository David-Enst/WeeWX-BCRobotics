import time
import datetime

#import sys	# Define path to virtual PYTHON libraries
#sys.path.extend(["/home/USER/VIRTUALENV/lib64/python3.11/site-packages"])
import sys	# Define path to virtual PYTHON libraries
sys.path.extend(["/home/dave/weeve/lib/python3.11/site-packages"])

import board
import busio
from adafruit_bme280 import basic as adafruit_bme280

from w1thermsensor import W1ThermSensor
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Use the button object to detect the wind speed and rain
from gpiozero import Device, Button

#
# BCRobotics Test App V5.11
#
out_temp = 0
windTick = 0     # Count of the wind speed input trigger
rainTick = 0     # Count of the rain input trigger
interval = 3     # Refresh readings every 3 seconds
rainTime = 0     # Use this to detect erroneous rain events
out_Temp = 0     # Set now to define scope for rain check
last_out = 0     # To handle temp sensor errors
TempSensor = False  # Temperature sensor flag
I2CSensor =  False  # I2C interface flag
WindSpSens = False  # Wind speed sensor flag
RainSensor = False  # Rain measurement sensor flag

# Setup Temperature sensor
try:
    ds18b20 = W1ThermSensor()
    time.sleep(interval)

except Exception as err:
    print ('Temperature Sensor Error: %s' % err)
    TempSensor = False
else:
    print ('W1ThermSensor setup fine.')
    TempSensor = True

# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)
 
# Setup wind direction ADC
try:
     ads = ADS.ADS1015(i2c)
     ads.gain = 1
     chan = AnalogIn(ads, ADS.P0)
except Exception as err:
     print ('Wind Direction setup Error: %s' % err)
     I2CSensor = False
else:
     print ('Wind Direction setup fine.')
     I2CSensor = True
     
# If BME setup gets an error then the wind dir won't work too
try:
     bme = adafruit_bme280.Adafruit_BME280_I2C(i2c) 
except Exception as err:
     print ('BME280 Error: %s' % err)
     I2CSensor = False
else:
     I2CSensor = True
     print ('BME280 setup fine.')

if I2CSensor :
    try:
        bme.sea_level_pressure = 1010.25 # generic value
        print ('Pressure and humidity setup fine.')
    except Exception as err:
        print ('Pressure and humidity sensor error.')

# Define event to detect wind (4 ticks per revolution)
#  1 tick/sec = 1.492 mph or 2.4 km/h
#  So: wind = (windTick * 2.4) / loop_interval
def windtrig(self):
    global windTick
    windTick += 1
    print ('Wind Tick.')

# Set digital pin 17 to an input (wind speed)
try:
    windEvent = Button(17)
except Exception as err:
    print ('Wind Speed Error: %s' % err)
    WindSpSens = False
else: 
    WindSpSens = True
    print ('Wind Speed setup fine.')
        

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
    print ('Rain Tick.')

# Set digital pin 23 to an input (rain)
try:
    rainEvent = Button(23)
except Exception as err:
    print ('Rain Error: %s' % err)
    RainSensor = False
else: 
    RainSensor = True
    print ('Rain setup fine.')

# Define event callbacks for 'pressed' 
if RainSensor :
    rainEvent.when_pressed = raintrig

while True:

    # Get Temperature, humidity & Pressure from BME280
    if I2CSensor : 
        case_temp = bme.temperature 
        pressure_pa= bme.pressure
        #pressure_pa = pressure_pa + 87/8.3 # adjust to sea level
        pressure = pressure_pa    # leave in hPa = mBar
        humidity = bme.humidity
        altitude = bme.altitude
    else:
        altitude = 0
        case_temp = 0
        pressure = 0
        humidity = 0
   
    # Get temperature from DS18B20 sensor
    if TempSensor : out_temp = ds18b20.get_temperature()
    else: out_temp = case_temp


    # Calculate wind direction based on ADC reading
    #   Read ADC channel 0 with a gain of 1
    if I2CSensor : val = chan.value
    else: val = 7000

    windDir = "No Sensor"

    if 19600 <= val <= 20999:
        windDir = "N"

    if 9000 <= val <= 10799:
        windDir = "NNE"

    if 10800 <= val <= 13999:
        windDir = "NE"

    if 2000 <= val <= 2299:
        windDir = "ENE"

    if 2300 <= val <= 2999:
        windDir = "E"

    if 1000 <= val <= 1999:
        windDir = "ESE"

    if 4000 <= val <= 4999:
        windDir = "SE"

    if 3000 <= val <= 3999:
        windDir = "SSE"

    if 6600 <= val <= 8999:
        windDir = "S"

    if 5000 <= val <= 6599:
        windDir = "SSW"

    if 15900 <= val <= 17499:
        windDir = "SW"

    if 14000 <= val <= 15899:
        windDir = "WSW"

    if 24000 <= val <= 24999:
        windDir = "W"

    if 21000 <= val <= 21999:
        windDir = "WNW"

    if 22000 <= val <= 23999:
        windDir = "NW"

    if 17500 <= val <= 19599:
        windDir = "NNW"

    # Calculate the average wind speed over 
    #   this 'interval' in km/h
    if WindSpSens :
        windSpeed = (windTick * 1.2) / interval
        windTick = 0
    else:
        windSpeed = 0

    #Calculate the rainfall over this 'interval' in mm
    if RainSensor :
        rainFall = rainTick * 0.2794
        rainTick = 0
    else:
        rainFall = 0
    
    ts = time.time()
    myTime = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    # Print results
    print("\nTime: " , myTime)
    print("Temperature: %0.1f C" % out_temp)
    print("Humidity: %0.1f %%" % humidity)
    print("Pressure: %0.2f hPa" % pressure)
    print("Case Temperature: %0.1f C" % case_temp) 
    print("Altitude = %0.2f meters" % altitude)
    print( 'Wind Dir:    ' , windDir, ' (', val, ')')
    print ("Wind Speed: %0.2f km/h" % windSpeed)
    print ("Rainfall:   %0.2f  mm" % rainFall)
    print (" ")
    time.sleep(interval)
