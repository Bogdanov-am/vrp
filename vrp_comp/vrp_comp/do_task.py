import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
import threading
import time
import vrp_comp.util as util
import math


class DoTask(Node):
    def __init__(self):
        super().__init__('do_task')
        self.right_ = self.create_publisher(
            Float64,
            '/booblik/thrusters/right/thrust',
            10)
        self.left_ = self.create_publisher(
            Float64,
            '/booblik/thrusters/left/thrust',
            10)
        self.back_ = self.create_publisher(
            Float64,
            '/booblik/thrusters/back/thrust',
            10)

        self.nav_ = self.create_subscription(
            NavSatFix,
            '/booblik/sensors/gps/navsat/fix',
            self.nav_callvack,
            10)
        self.nav_

        self.imu_ = self.create_subscription(
            Imu,
            '/booblik/sensors/imu/imu/data',
            self.imu_callback,
            10)
        self.imu_

        self.smooth_count = 5

        self.quaternion = None
        self.coords = []

        self.cog = 0
        self.speed = 0

    def nav_callvack(self, msg):
        self.coords.append([msg.latitude, msg.longitude])
        if len(self.coords) > 5:
            self.coords.pop(0)

        if len(self.coords) > 1:
            self.cog, self.speed = util.calculate_cog_and_speed(self.coords, 0.1)
            # print('Cog:', math.degrees(self.cog))

    def imu_callback(self, msg):
        self.quaternion = msg.orientation
        roll, pitch, yaw = util.euler_from_quaternion(
            self.quaternion.x,
            self.quaternion.y,
            self.quaternion.z,
            self.quaternion.w,
        )
        print('Angles: {:6.2f}, {:6.2f}, {:6.2f}'.format(roll, pitch, -yaw + math.pi/2))

    def task(coords, cog, hdg, speed, img):
        pass


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = DoTask()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.stop()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()
