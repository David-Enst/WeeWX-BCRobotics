import time
import board
import busio
from adafruit_bme280 import basic as adafruit_bme280
#import adafruit_bme280

from w1thermsensor import W1ThermSensor
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

import RPi.GPIO as GPIO


out_temp = 0

try:
    ds18b20 = W1ThermSensor()
    print ('DS18b20 temp sensor OK')
    TempSensor = True
except Exception as err:
    print ('Temperature Sensor Error:', err)
    TempSensor = False

# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)

try:
     bme = adafruit_bme280.Adafruit_BME280_I2C(i2c) # temp, pressure, humidity
     PressSensor = True
except Exception as err:
     print ('BME280 Pressure Error:', err)
     PressSensor = False
 
try:
     ads = ADS.ADS1015(i2c)        # wind and rain
     ads.gain = 1
     chan = AnalogIn(ads, ADS.P0)
     WindSensor = True
except Exception as err:
     print ('Wind Sensor Error:', err)
     WindSensor = False

if PressSensor :
    try:
        bme.sea_level_pressure = 1025.25 # generic value
    except Exception as err:
        loginf('Pressure Sensor Error: %s' % err)


interval = 8   # Time between loops (seconds)
windTick = 0   # Count of the wind speed input trigger
rainTick = 0   # Count of the rain input trigger
val      = 0   # Wind direction

# Set GPIO pins to use BCM pin numbers
GPIO.setmode(GPIO.BCM)

# Set digital pin 17 to an input and enable the pullup (wind speed)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set digital pin 23 to an input and enable the pullup (rain)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Event to detect wind (4 ticks per revolution)
GPIO.add_event_detect(17, GPIO.BOTH)
def windtrig(self):
    global windTick
    windTick += 1

GPIO.add_event_callback(17, windtrig)

# Event to detect rain (0.2794mm per tick)
GPIO.add_event_detect(23, GPIO.FALLING)
def raintrig(self):
    global rainTick
    rainTick += 1

GPIO.add_event_callback(23, raintrig)

while True:
   
    # Get temperature from DS18B20 sensor
    if TempSensor : out_temp = ds18b20.get_temperature()

    # Get Temperature, humidity & Pressure from BME280
    if PressSensor : 
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


    # Calculate wind direction based on ADC reading
    #   Read ADC channel 0 with a gain of 1
    if WindSensor : val = chan.value

    windDir = "Not connected"

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
    windSpeed = (windTick * 1.2) / interval
    windTick = 0

    #Calculate the rainfall over this 'interval' in mm
    rainFall = rainTick * 0.2794
    rainTick = 0
    
    # Print results
    print("\nTemperature: %0.1f C" % out_temp)
    print("Humidity: %0.1f %%" % humidity)
    print("Pressure: %0.2f hPa" % pressure)
    print("Case Temperature: %0.1f C" % case_temp) 
    print("Altitude = %0.2f meters" % altitude)
    
    print( 'Wind Dir:    ' , windDir, ' (', val, ')')
    print ("Wind Speed: %0.2f km/h" % windSpeed)
    print ("Rainfall:   %0.2f  mm" % rainFall)
    print (" ")
    time.sleep(interval)
    

