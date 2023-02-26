import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
import threading
import time
import vrp_comp.util as util
import math


class DoTask(Node):
    def __init__(self, name='do_task', period=0.1):
        super().__init__(name)
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

        self.yaw_ = self.create_publisher(
            Float64,
            '/booblik/yaw',
            10)
        self.cog_ = self.create_publisher(
            Float64,
            '/booblik/cog',
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

        self.period = period

        self.smooth_count = 5

        self.quaternion = None
        self.coords = []
        self.latitude = 0
        self.longitude = 0

        self.cog = 0
        self.speed = 0
        self.yaw = 0

    def start(self):
        self.taskThread = threading.Thread(
            target=self._taskLoop, daemon=True).start()

    def nav_callvack(self, msg):
        self.coords.append([msg.latitude, msg.longitude])
        self.latitude = msg.latitude
        self.longitude = msg.longitude

        if len(self.coords) > 5:
            self.coords.pop(0)

        if len(self.coords) > 1:
            self.cog, self.speed = util.calculate_cog_and_speed(
                self.coords, 0.1)

            cog = Float64()
            cog.data = math.degrees(self.cog)
            self.cog_.publish(cog)

    def imu_callback(self, msg):
        self.quaternion = msg.orientation
        roll, pitch, yaw = util.euler_from_quaternion(
            self.quaternion.x,
            self.quaternion.y,
            self.quaternion.z,
            self.quaternion.w,
        )
        self.yaw = -yaw + math.pi/2

        if self.yaw > math.pi:
            self.yaw = self.yaw - 2 * math.pi
        elif self.yaw < -math.pi:
            self.yaw = self.yaw + 2 * math.pi

        yaw = Float64()
        yaw.data = math.degrees(self.yaw)
        self.yaw_.publish(yaw)

    def _taskLoop(self):
        while True:
            res = self.task(
                self.latitude,
                self.longitude,
                math.degrees(self.cog),
                math.degrees(self.yaw),
                self.speed,
                None
            )
            self.send_thrust(res)
            time.sleep(self.period)

    def task(self, latitude, longitude, cog, yaw, speed, img):
        return {
            'mode': util.ThrustMode.Direct_Mode,
            'l': 0,
            'r': 0,
            'b': 0
        }

    def send_thrust(self, target):
        l, r, b = [.0, .0, .0]

        if target ==  None:
            pass
        elif target['mode'] == util.ThrustMode.Vector_Mode:
            l, r, b = util.vector_thrust_decomposition(
                target['x'], target['y'], target['z'])
        elif target['mode'] == util.ThrustMode.Direct_Mode:
            l, r, b = [target['l'], target['r'], target['b']]

        left = Float64()
        left.data = l * 20.0
        right = Float64()
        right.data = r * 20.0
        back = Float64()
        back.data = b * 20.0

        self.right_.publish(right)
        self.left_.publish(left)
        self.back_.publish(back)


def main(args=None):
    rclpy.init(args=args)
    task = DoTask()
    task.start()
    rclpy.spin(task)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
