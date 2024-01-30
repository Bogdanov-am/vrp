import math
import struct
from dataclasses import dataclass
import serial
import numpy as np
from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@dataclass
class LidarConfig:
    port: str   # Порт подключения
    baudrate: int   # Скорость передачи данных


class LidarNode(Node):
    def __init__(self, config: LidarConfig, name='lidar'):
        super().__init__(name)
        self.config = config
        self.freq = 10  # Частота вращения измерений лидара в Гц
        # Создание издателя для публикации данных лидара
        self.lidar_ = self.create_publisher(
            LaserScan,
            '/booblik/sensors/lidar',
            10)
        self.lidar_

    def start(self):
        # Запуск потока для чтения данных с лидара
        Thread(target=self._readLoop, daemon=False).start()

    # Приватный метод для чтения данных с лидара
    def _readLoop(self):
        ser = serial.Serial(
            self.config.port,
            self.config.baudrate
        )
        buffer = bytearray()
        # Установка частоты вращения лидара с помощью команды
        ser.write('LSRPM:{:d}H\n\r'.format(self.freq*60).encode('ascii'))
        ser.write('LOCONH\n\r'.format(self.freq*60).encode('ascii'))

        # Кольцевой буфер для чтения пакетов байт с последовательного порта
        while True:
            buffer.extend(ser.read(256))
            index = buffer.find(bytearray([0xCE, 0xFA]), 1)
            while index != -1 and index:
                packet = buffer[0:index]
                self._parsePacket(packet)
                buffer = buffer[index:]
                index = buffer.find(bytearray([0xCE, 0xFA]), 1)

    def _parsePacket(self, packet: bytearray):
        # Проверка на минимально допустимую длину пакета
        if len(packet) <= 10:
            print('too low length')
            return

        # Распаковка начала пакета, его длины и угла в формате "unsigned short"
        [_start, length, angle] = struct.unpack('=HHH', packet[0:6])
        # Проверка, что длина пакета соответствует ожидаемой
        if (len(packet) == 8+length*2):
            # Распаковка данных из пакета. Формат строки определяется длиной данных.
            data = struct.unpack('={}H'.format(length), packet[6:-2])
            # Извлечение контрольной суммы из последних двух байт пакета
            crc_in, = struct.unpack('=H'.format(length), packet[-2:])
            distances = np.zeros((length))
            strengths = np.zeros((length))
            # Расчет углов для каждого измерения
            angles = np.radians(np.linspace(angle/10, angle/10+36, length, endpoint=False))
            # Вычисление контрольной суммы для проверки целостности данных
            crc_calc = angle + length
            for i in range(length):
                crc_calc += data[i]
                crc_calc %= 65536
                # Расчет расстояния и интенсивности сигнала
                # Маскирование и масштабирование данных расстояния
                distances[i] = (data[i] & 0x1FFF) / 100
                # Извлечение интенсивности сигнала
                strengths[i] = (data[i] & 0xe000) >> 13
            if crc_in != crc_calc:  # Проверка контрольной суммы
                print("error crc")
                return
            else:  # Публикация данных сканирования, если данные корректны
                # Создание объекта LaserScan для публикации данных
                laser_scan = LaserScan()
                laser_scan.angle_min = angles[0]  # Минимальный угол сканирования
                laser_scan.angle_max = angles[-1]  # Максимальный угол сканирования
                laser_scan.angle_increment = angles[1] - angles[0]  # Шаг угла сканирования
                laser_scan.scan_time = 1.0 / self.freq  # Время сканирования
                laser_scan.time_increment = 1.0 / self.freq / 10 / \
                    length  # Временной интервал между измерениями
                laser_scan.range_min = 0.  # Минимальное расстояние сканирования
                laser_scan.range_max = 30.  # Максимальное расстояние сканирования
                laser_scan.ranges = distances.tolist()  # Список измеренных расстояний
                laser_scan.intensities = strengths.tolist()  # Список интенсивностей
                # Публикация данных сканирования в  ROS топик
                self.lidar_.publish(laser_scan)


def main(args=None):
    rclpy.init(args=args)
    task = LidarNode(LidarConfig('/dev/ttyUSB0', 230400))
    task.start()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
