import numpy as np
from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarShowNode(Node):
    def __init__(self, name='lidar_show'):
        super().__init__(name)
        self.create_subscription(
            LaserScan,
            '/booblik/sensors/lidar',
            self._lidar_callback,
            10)
        # Инициализация данных для визуализации
        self.cache_theta = np.array([])
        self.cashe_r = np.array([])
        
        # Настройка отображения графика в полярных координатах
        _, ax = plt.subplots(subplot_kw={'polar': True})
        self.line, = ax.plot(self.cache_theta, self.cashe_r, 'ro', markersize=1)
        ax.set_ylim((0, 3))
        plt.get_current_fig_manager().set_window_title('Лидар')
        ax.set_yticks(np.arange(0, 3, 0.5))
        ax.set_xticks(np.linspace(0, 2*np.pi, 36, endpoint=False))
        ax.set_theta_offset(np.pi / 2)

    def _lidar_callback(self, msg: LaserScan):
        # Кэширование данных для отображения их на графике
        length = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, length)
        
        self.cache_theta = np.concatenate([self.cache_theta, angles])
        self.cashe_r = np.concatenate([self.cashe_r, msg.ranges])

        # Очистка кэша если прошло одно полное вращение
        if msg.angle_min == 0:
            self.line.set_xdata(-self.cache_theta)
            self.line.set_ydata(self.cashe_r)
            self.cache_theta = []
            self.cashe_r = []
            plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    task = LidarShowNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
