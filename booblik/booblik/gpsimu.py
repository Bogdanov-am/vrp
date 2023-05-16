# import troykahat
import struct
from dataclasses import dataclass
import serial
from threading import Thread
import rclpy
from rclpy.node import Node
from std_msgs.msg import NavSatFix


format_time = 'BBBB'
format_latlon = "=BBiic"
format_angle = "=BBhhhhc"
format_quat = '=BBhhhhc'


def parseLanLon(packet: bytearray):
    s = struct.unpack(format_latlon, packet)
    lon = s[2]//1e7 + (s[2] % 1e7)/1e5/60
    lat = s[3]//1e7 + (s[3] % 1e7)/1e5/60
    print(lat, lon)
    return lat, lon


def parseAngle(packet: bytearray):
    s = struct.unpack(format_angle, packet)
    roll = s[2] / 32768 * 180
    pitch = s[3] / 32768 * 180
    yaw = s[4] / 32768 * 180
    return roll, pitch, yaw


def parseAngle(packet: bytearray):
    s = struct.unpack(format_angle, packet)
    roll = s[2] / 32768 * 180
    pitch = s[3] / 32768 * 180
    yaw = s[4] / 32768 * 180
    return roll, pitch, yaw


def parseQuat(packet: bytearray):
    s = struct.unpack(format_angle, packet)
    q0 = s[2] / 32768
    q1 = s[3] / 32768
    q2 = s[4] / 32768
    q3 = s[5] / 32768
    return q0, q1, q2, q3


@dataclass
class GpsConfig:
    port: str
    baudrate: int


class GpsImuNode(Node):
    config: GpsConfig

    def __init__(self, name='gpsimu'):
        super().__init__(name)
        self.config = GpsConfig('COM4', 9600)
        self.left = 0
        self.right = 0
        self.back = 0

        self.nav_ = self.create_publisher(
            NavSatFix,
            '/booblik/sensors/gps/navsat/fix',
            10)
        self.nav_

        Thread(target=self._readLoop, daemon=True).start()

    def _readLoop(self):
        ser = serial.Serial(
            self.config.port,
            self.config.baudrate
        )  # open serial port
        buffer = bytearray()

        while True:
            buffer.extend(ser.read(22))
            index = buffer.find(0x55, 1)
            while index != -1 and index:
                packet = buffer[0:index]
                if (len(packet) == 11):
                    self.parsePacket(packet)
                buffer = buffer[index:]
                index = buffer.find(0x55, 1)

    def parsePacket(self, packet: bytearray):
        if packet[1] == 0x59:
            q0, q1, q2, q3 = parseQuat(packet)
        elif packet[1] == 0x57:
            lat, lon = parseLanLon(packet)
            nav = NavSatFix()
            nav.latitude = lat
            nav.longitude = lon
            self.nav_.publish(nav)


def main(args=None):
    rclpy.init(args=args)
    task = GpsImuNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
