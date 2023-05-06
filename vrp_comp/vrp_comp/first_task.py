import rclpy
from vrp_comp.do_task import DoTask


class DoFirstTask(DoTask):
    def __init__(self):
        super().__init__('first_task')
        self.declare_parameter('target_angle', 90.0)
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

        # target_angle - целевой угол в градусах
        target_angle = self.get_parameter(
            'target_angle').get_parameter_value().double_value

        print('Текущий угол: {:3.0f}, целевой угол: {:3.0f}'.format(hdg, target_angle))
        
        # РАСПОЛОЖИТЕ ВАШ КОД ДАЛЕЕ
        
        # Пример простейшего релейного регулятора угла.
        # Вычисляем разницу между текущим курсом и целевым углом
        delta_angle = target_angle - hdg 
        if delta_angle > 180:
            delta_angle = delta_angle - 360
        elif delta_angle < -180:
            delta_angle = delta_angle + 360

        threshold = 2.5 # пороговый угол
        thrust = 0.05 # значение мощности, когда target_angle больше threshold

        z = 0
        if delta_angle > threshold:
            z = thrust
        elif delta_angle < -threshold:
            z = -thrust
            
        return {'mode': 0, 'l': z, 'r': z, 'b': z}


def main(args=None):
    rclpy.init(args=args)
    task = DoFirstTask()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
