import time
from w1thermsensor import W1ThermSensor
from Adafruit_BME280 import *
import Adafruit_ADS1x15
import RPi.GPIO as GPIO

try:
     ds18b20 = W1ThermSensor()
except Exception as err:
    print ('Error:', err)
    noTemp = True
else:
    print ('carry on')
    noTemp = False

bme = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8)
adc = Adafruit_ADS1x15.ADS1115()

interval = 2   # Time between loops (seconds)
windTick = 0   # Count of the wind speed input trigger
rainTick = 0   # Count of the rain input trigger

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
    time.sleep(interval)
    
    # Get temperature from DS18B20 sensor
    temperature = ds18b20.get_temperature()

    # Get Temperature from BME280
    case_temp = bme.read_temperature()

    # Get Barometric Pressure from BME280 and convert to kPa from pascals
    pressure_pa= bme.read_pressure()
    pressure = pressure_pa / 1000

    # Get Humidity from BME280
    humidity = bme.read_humidity()

    # Calculate wind direction based on ADC reading
    #   Read ADC channel 0 with a gain of 1
    val = adc.read_adc(0, gain=1)

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

    if 15900 <= val <= 16999:
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
    print 'Temperature: ', temperature, 'C'
    print 'Humidity:    ', humidity, '%'
    print 'Case Temp:   ', case_temp, 'C'
    print 'Pressure:    ', pressure, 'kPa'
    print 'Dir ADC val: ', val
    print 'Wind Dir:    ', windDir
    print 'Wind Speed:  ', windSpeed, 'km/h'
    print 'Rainfall:    ', rainFall, 'mm'
    print ' '
    
