from dataclasses import dataclass
from brping import Ping1D
from threading import Thread
import rclpy
from rclpy.node import Node
from booblik_msg.msg import Echo1d
import time
from datetime import datetime

@dataclass
class Echo1dConfig:
    port: str
    baudrate: int
    period: float # ms
    speed_of_sound: float # m/s


class Echo1dNode(Node):
    config: Echo1dConfig

    def __init__(self, name='echo_ping'):
        super().__init__(name)
        self.config = Echo1dConfig(
            '/dev/ttyUSB0', 
            115200,
            500,
            1490
        )
        self.echo1d_ = self.create_publisher(
            Echo1d,
            '/booblik/sensors/echo1d',
            10)

        Thread(target=self._readLoop, daemon=True).start()

    def _readLoop(self):
        ping = Ping1D()
        ping.connect_serial(self.config.port, self.config.baudrate)
        if ping.initialize() is False:
            print("Failed to initialize Ping!")
            exit(1)
        ping.set_speed_of_sound(self.config.speed_of_sound * 1000)

        while True:
            start = datetime.now()
            data = ping.get_distance()
            msg = Echo1d()
            msg.distance = data["distance"]
            msg.confidence = data["confidence"]
            msg.scan_start = data["scan_start"]
            msg.scan_length = data["scan_length"]
            msg.transmit_duration = data["transmit_duration"]
            msg.gain_setting = data["gain_setting"]

            self.echo1d_.publish(msg)
            self.get_logger().info("Distance: %s\tConfidence: %s%%" % (data["distance"], data["confidence"]))
            time.sleep(self.config.period/1000)



def main(args=None):
    rclpy.init(args=args)
    task = Echo1dNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
