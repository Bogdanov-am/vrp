from dataclasses import dataclass
import serial
from threading import Thread
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pynmea2


@dataclass
class GpsConfig:
    port: str
    baudrate: int


class GpsImuNode(Node):
    config: GpsConfig

    def __init__(self, name='ws_m181'):
        super().__init__(name)
        self.config = GpsConfig('/dev/serial0', 115200)
        self.nav_ = self.create_publisher(
            NavSatFix,
            '/booblik/sensors/gps/navsat/fix',
            10)
        Thread(target=self._readLoop, daemon=True).start()


    def _readLoop(self):
        ser = serial.Serial(
            self.config.port,
            self.config.baudrate,
            timeout=3
        )  # open serial port
        while True:
            try:
                raw_data = ser.readline().decode()#read data
                data = pynmea2.parse(raw_data)#parse data
                
                #match input nmea packet
                if data.sentence_type == "GGA":
                    nav = NavSatFix()
                    nav.latitude = data.latitude
                    nav.longitude = data.longitude
                    self.nav_.publish(nav)
                    print(data.latitude, " ", data.longitude)
            except Exception as e:
                pass


def main(args=None):
    rclpy.init(args=args)
    task = GpsImuNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
