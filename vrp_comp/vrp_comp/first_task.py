import rclpy
from vrp_comp.do_task import DoTask


class DoFirstTask(DoTask):
    def __init__(self):
        super().__init__('first_task')
        self.declare_parameter('target_angle', 'fff')
        self.start()

    def task(self, latitude, longitude, cog, yaw, speed, img):
        """
        :param latitude: Географическая широта в градусах от -90 до 90; Float
        :param longitude: географисчкая долгота в градусах от -180 до 180; Float
        :param cog: Курс; Float
        :param yaw: ; Float
        :param speed: ; Float
        :return: Thrust on each thruster (l, r, b) ; (-1,1); Float
        """

        # target_angle - целевой угол в градусах
        target_angle = self.get_parameter(
            'target_angle').get_parameter_value().double_value

        return None


def main(args=None):
    rclpy.init(args=args)
    task = DoFirstTask()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
