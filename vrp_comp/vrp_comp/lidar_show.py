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
        self.cache_range = np.array([])
        self.cache_intensities = np.array([])

        # Настройка отображения графика в полярных координатах
        with plt.style.context('dark_background'):
            _, ax = plt.subplots(subplot_kw={'polar': True})
            self.sc = ax.scatter(
                self.cache_theta,
                self.cache_range,
                c=self.cache_intensities,
                cmap='autumn', s=5)
            max_range = 3
            ax.set_ylim((0, max_range))
            plt.get_current_fig_manager().set_window_title('Лидар')
            cbar = plt.colorbar(self.sc, shrink=0.7, aspect=20)
            self.sc.set_clim(0, 1)
            
            ax.set_xticks(np.linspace(0, 2*np.pi, 36, endpoint=False))
            ax.set_theta_offset(np.pi / 2)
            ax.set_theta_direction(-1)
            plt.subplots_adjust(left=0.06, right=0.94, top=0.94, bottom=0.06)

    def _lidar_callback(self, msg: LaserScan):
        # Кэширование данных для отображения их на графике
        length = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, length)

        self.cache_theta = np.concatenate([self.cache_theta, angles])
        self.cache_range = np.concatenate([self.cache_range, msg.ranges])
        self.cache_intensities = np.concatenate(
            [self.cache_intensities, msg.intensities])

        # Очистка кэша если прошло одно полное вращение
        if msg.angle_min == 0:
            self.sc.set_offsets(np.c_[self.cache_theta, self.cache_range])
            self.sc.set_array(self.cache_intensities/5)
            self.cache_theta = []
            self.cache_range = []
            self.cache_intensities = []
            plt.pause(0.001)
            


def main(args=None):
    rclpy.init(args=args)
    task = LidarShowNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
