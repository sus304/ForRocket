import numpy as np

class AreaWind:
    def __init__(self, area_config):
        wind_config = area_config.get('Law Wind')

        self.wind_exponatial = wind_config.get('Wind Characteristic Coefficient')
        self.height_wind_reference = wind_config.get('Reference Height [m]')

        self.wind_speed_min = wind_config.get('Reference Wind Speed Lower Limit [m/s]')
        self.wind_speed_max = wind_config.get('Reference Wind Speed Upper Limit [m/s]')
        self.wind_speed_step = wind_config.get('Reference Wind Speed Step [m/s]')
        if self.wind_speed_min > self.wind_speed_max:
            print('Error! Wind speed upper limit bigger than lower limit.')
            print('Configration: Upper: ', self.wind_speed_max, ' Lower: ', self.wind_speed_min)
            exit()
        if self.wind_speed_step <= 0.0:
            print('Error! Wind speed step bigger than 0.')
            exit()
        self.wind_speed_array = np.arange(self.wind_speed_min, self.wind_speed_max+self.wind_speed_step, self.wind_speed_step)

        self.wind_direction_min = wind_config.get('Wind Direction Lower Limit [deg]')
        self.wind_direction_max = wind_config.get('Wind Direction Upper Limit [deg]')
        self.wind_direction_step = wind_config.get('Wind Direction Step [deg]')
        if self.wind_direction_min > self.wind_direction_max:
            print('Error! Wind direction upper limit bigger than lower limit.')
            print('Configration: Upper: ', self.wind_direction_max, ' Lower: ', self.wind_direction_min)
            exit()
        if self.wind_direction_step <= 0.0:
            print('Error! Wind direction step bigger than 0.')
            exit()
        if self.wind_direction_max >= 360.0:
            self.wind_direction_max = 360.0 - self.wind_direction_step
        self.wind_direction_array = np.arange(self.wind_direction_min, self.wind_direction_max+self.wind_direction_step, self.wind_direction_step)

    def make_law_wind(self, height_ref, vel_ref, dir_ref, exp_a):
        def _law_method(alt):
            return vel_ref * (alt / height_ref) ** (1 / exp_a)

        alt_max = 20000.0  # 20 km

        base = np.cos(np.arange(0.0, 0.5*np.pi, 0.005))
        alt_array = alt_max * (1 - base)
        vel_array = _law_method(alt_array)
        vel_u_array = -1.0 * vel_array * np.cos(np.radians(dir_ref))
        vel_v_array = -1.0 * vel_array * np.sin(np.radians(dir_ref))

        return alt_array, vel_u_array, vel_v_array
