import numpy as np
from LaunchSiteData.coordinate import *


class LaunchSite:
    def magnetic_declination(self):
        lat = self.launch_point_LLH[0]
        lon = self.launch_point_LLH[1]

        delta_lat = lat - 37.0
        delta_lon = lon - 138.0
        mag_dec = (7.0 + 57.201 / 60.0)	+ (18.750 / 60.0) * delta_lat - (6.761 / 60.0) * delta_lon - (0.059 / 60.0) * delta_lat ** 2 - (0.014 / 60.0) * delta_lat * delta_lon - (0.579 / 60.0) * delta_lon ** 2
        return mag_dec
    
    def wind_law(self, alt, ref_wind_speed, ref_altitude):
        speed = ref_wind_speed * (alt / ref_altitude) ** (1.0 / self.wind_power_exp)
        return speed

    

    


