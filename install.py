# installer for the weewx-bcrobo driver
# Copyright 2019 David Enstrom, all rights reserved
#
# See:    https://www.sparkfun.com/products/8942
#         https://www.bc-robotics.com/tutorials/raspberry-pi-weather-station-part-1/
#
# NOTE: We have moved to the 21st Century, so the database is in METRIC!
#       Therefore, if you have a database created already, then see:
#         http://weewx.com/docs/customizing.htm#Changing_the_unit_system

# installer for the weewx-bcrobo driver
# Copyright 2019 David Enstrom, all rights reserved
#
# See:    https://www.sparkfun.com/products/8942
#         https://www.bc-robotics.com/tutorials/raspberry-pi-weather-station-part-1/
#
# NOTE: We have moved to the 21st Century, so the database is in METRIC!
#       Therefore, if you have a database created already, then see:
#         http://weewx.com/docs/customizing.htm#Changing_the_unit_system

from weecfg.extension import ExtensionInstaller


def loader():
    return BCRoboInstaller()


class BCRoboInstaller(ExtensionInstaller):
    def __init__(self):
        super(BCRoboInstaller, self).__init__(
            name='BCRobotics',
            description='Get weather data from BC Robotics',
            author="David Enstrom",
            author_email="denstrom.de@gmail.com",
            version="1.4",
            files=[('bin/user', ['bin/user/BCRobotics.py'])]
            )
