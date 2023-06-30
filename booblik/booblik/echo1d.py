from dataclasses import dataclass
from brping import Ping1D
from threading import Thread
import rclpy
from rclpy.node import Node
from booblik_msg.msg import Echo1d

@dataclass
class Echo1dConfig:
    port: str
    baudrate: int


class Echo1dNode(Node):
    config: Echo1dConfig

    def __init__(self, name='gpsimu'):
        super().__init__(name)
        self.config = Echo1dConfig('COM4', 9600)
        self.echo1d_ = self.create_publisher(
            Echo1d,
            '/booblik/sensors/echo1d',
            10)
        self.echo1d_

        Thread(target=self._readLoop, daemon=True).start()

    def _readLoop(self):
        myPing = Ping1D()
        myPing.connect_serial("/dev/ttyUSB0", 115200)
        if myPing.initialize() is False:
            print("Failed to initialize Ping!")
            exit(1)

        print(myPing.get_device_id())
        print(myPing.get_device_information())
        print(myPing.get_distance())
        print(myPing.get_distance_simple())
        print(myPing.get_gain_setting())
        print(myPing.get_firmware_version())
        print(myPing.get_general_info())
        print(myPing.get_mode_auto())
        print(myPing.get_pcb_temperature())
        print(myPing.get_ping_enable())
        print(myPing.get_ping_interval())
        print(myPing.get_profile())
        print(myPing.get_processor_temperature())
        print(myPing.get_speed_of_sound())
        print(myPing.get_range())
        print(myPing.get_transmit_duration())
        print(myPing.get_protocol_version())
        # if data:
        #     print(data)
        #     # print("Distance: %s\tConfidence: %s%%" % (data["distance"], data["confidence"]))
        # else:
        #     print("Failed to get distance data")


def main(args=None):
    rclpy.init(args=args)
    task = Echo1dNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
