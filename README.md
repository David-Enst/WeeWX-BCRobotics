# WeeWX-BCRobotics
A WeeWX driver for the BC Robotics and SparkFun weather station Raspberry Pi based hardware.

weewx-bcrobo

Driver to collect data from the "Spark Fun" SEN-08942 RoHS weather meters 
connected to the BC Robotics interface connected to a Raspberry Pi.

  NOTE: I have also written an Android app to display WeeWX readings, see:
        https://sites.google.com/view/mywx/home 
        Or just search for "MyWX" on the Google Play store.

SEE:
https://www.sparkfun.com/products/8942
https://www.bc-robotics.com/tutorials/raspberry-pi-weather-station-part-1/

INTRODUCTION

The BC Robotics interface uses the Aidafruit BME280 sensor for humidity, pressure, 
and temperature along with a ADS1x15 ADC (analog to digital converter) to read the 
wind direction. The rain and wind speed are measured through the GPIO interface by 
counting "ticks" (rotations or measures of water). The driver includes a erroneous
rain tick filter, since this sensor can emit single undesired ticks.

If you want to build your own system see the documentation provided:
 1 - Weather Station Raspberry Pi Construction.pdf
 2 - Weather Station Raspberry Pi Software Setup for the MyWX App.pdf
 3 - Weather Station Raspberry Pi Security Setup for MyWX Pro.pdf 

SOFTWARE and SECURITY SETUP

Installing WeeWX, required software and the BC Robotics driver is defined in #2 above.
The required SSL security setup is defined in #3 above.

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
