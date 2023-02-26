import rclpy
from vrp_comp.do_task import DoTask
from vrp_comp.util import R_earth
import sys
import math

class DoSecondTask(DoTask):
    def __init__(self):
        super().__init__('second_task')
        self.declare_parameter('longitudes', [150.6739099669496, 150.67403880435694, 150.6741587316573, 150.6740302454607])
        self.declare_parameter('latitudes', [-33.72252723805997, -33.72241893456141, -33.72253715614862, -33.72264554139414])
        self.start()

    def task(self, latitude, longitude, cog, hdg, speed, img):
        """
        :param latitude: географическая широта в градусах от -90 до 90; Float
        :param longitude: географическая долгота в градусах от -180 до 180; Float
        :param cog: Направление скорости относительно севера в градусах от -180 до 180; Float
        :param hdg: Направление носа относительно севера в градусах от -180 до 180; Float
        :param speed: Скорость в метрах в секунду; Float
        :return: Мощность на каждом двигателе {'mode': 0, 'l': (-1,1), 'r': (-1,1), 'b': (-1,1)}
            или направление {'mode': 1, 'x': (-1,1), 'y': (-1,1), 'z': (-1,1)}
        """

        latitudes = self.get_parameter(
            'latitudes').get_parameter_value().double_array_value
        longitudes = self.get_parameter(
            'longitudes').get_parameter_value().double_array_value
        
        # coordinates - массив целевых координат
        coordinates = []
        if len(latitudes) != len(longitudes):
            sys.exit(1)
        else:
            for i in range(len(latitudes)):
                coordinates.append([latitudes[i], longitudes[i]])
        
        # РАСПОЛОЖИТЕ ВАШ КОД ДАЛЕЕ
        # Вычисляем курс на точку 
        delta_lat = math.radians(coordinates[0][0] - latitude)
        delta_lon = math.radians(coordinates[0][1] - longitude)
        lat_avg = math.radians((latitude + coordinates[0][0]) / 2)

        delta_y = (delta_lat) 
        delta_x = (delta_lon) * math.cos(lat_avg)
        
        target_cog = math.degrees(math.atan2(delta_x, delta_y))

        print('Расстояние до точки: {:4.1f}'.format(math.sqrt(delta_x**2 + delta_y**2) * R_earth))

        # Алгоритм из первого задания (вместо hdg теперь cog)
        delta_angle = target_cog - cog 
        if delta_angle > 180:
            delta_angle = delta_angle - 360
        elif delta_angle < -180:
            delta_angle = delta_angle + 360

        threshold = 5 # пороговый угол
        forward = 0.6
        thrust = 0.1 # значение мощности, когда target_angle больше threshold

        z = 0
        if delta_angle > threshold:
            z = thrust
        elif delta_angle < -threshold:
            z = -thrust

        return {'mode': 0, 'l': forward, 'r': -forward, 'b': z}


def main(args=None):
    rclpy.init(args=args)
    task = DoSecondTask()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
