# installer for the weewx-bcrobo driver
# Copyright 2019 David Enstrom, all rights reserved
#
# See:    https://www.sparkfun.com/products/8942
#         https://www.bc-robotics.com/tutorials/raspberry-pi-weather-station-part-1/
#
# NOTE: We have moved to the 21st Century, so the database is in METRIC!
#       Therefore, if you have a database created already, then see:
#         http://weewx.com/docs/customizing.htm#Changing_the_unit_system

from setup import ExtensionInstaller

def loader():
    return BCRoboInstaller()

class BCRoboInstaller(ExtensionInstaller):
    def __init__(self):
        super(BCRoboInstaller, self).__init__(
            version="0.0.6",
            name='BCRobotics',
            description='Capture weather data from BC Robotics',
            author="David Enstrom",
            author_email="denstrom@rogers.com",
            config={
                'StdArchive': {
                    'record_generation': 'software'},
                'StdConvert': {
                    'target_unit': 'METRIC'},
                'StdQC': {
                    'MinMax': {
                        'barometer': '800, 1110, mbar',
                        'pressure': '800, 1110, mbar',
                        'outTemp': '-50, 49, degree_C',
                        'inTemp': '-50, 49, degree_C',
                        'windSpeed': '0, 300, km_per_hour'}},
                'StdReport': {
                    'StandardReport': {
                        'Units': {
                            'Groups': {
                                'group_altitude': 'meter',
                                'group_speed': 'km_per_hour',
                                'group_speed2': 'km_per_hour2',
                                'group_pressure': 'hPa',
                                'group_rain': 'mm',
                                'group_rainrate': 'mm_per_hour',
                                'group_temperature': 'degree_C',
                                'group_degree_day': 'degree_C',
                                'group_speed': 'km_per_hour'}}}}},

            files=[('bin/user', ['bin/user/BCRobotics.py'])]
        )
