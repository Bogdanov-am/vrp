import rclpy
from vrp_comp.do_task import DoTask
import sys

class DoSecondTask(DoTask):
    def __init__(self):
        super().__init__('second_task')
        self.declare_parameter('latitudes', [0.0])
        self.declare_parameter('longitudes', [0.0])
        self.start()

    def task(self, latitude, longitude, cog, hdg, speed, img):
        """
        :param latitude: географическая широта в градусах от -90 до 90; Float
        :param longitude: географическая долгота в градусах от -180 до 180; Float
        :param cog: Направление скорости относительно севера в градусах от -180 до 180; Float
        :param hdg: Направление носа относительно севера в градусах от -180 до 180; Float
        :param speed: Скорость в метрах в секунду; Float
        :return: Мощность на каждом двигателе {'mode': 1, 'l': (-1,1), 'r': (-1,1), 'b': (-1,1)}
            или направление {'mode': 0, 'x': (-1,1), 'y': (-1,1), 'z': (-1,1)}
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
        
        return {'mode': 1, 'l': 0, 'r': 0, 'b': 0}


def main(args=None):
    rclpy.init(args=args)
    task = DoSecondTask()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
