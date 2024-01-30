import math
import struct
import signal
import sys
from dataclasses import dataclass
import serial
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from threading import Event, Thread

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
        self.freq = 10 # Частота вращения измерений лидара в Гц
        # Создание издателя для публикации данных лидара
        self.lidar_ = self.create_publisher(
            LaserScan,
            '/booblik/sensors/lidar',
            10)
        self.lidar_
        self.is_run = Event() # Событие для управления потоком чтения

    # Метод для запуска измерений
    def start(self):
        # Инициализация данных для визуализации
        self.cache_theta = np.array([])
        self.cashe_r = np.array([])
        self.theta = np.array([])
        self.r = np.array([])
        
        # Настройка отображения графика в полярных координатах
        fig, ax = plt.subplots(subplot_kw={'polar': True})
        line, = ax.plot(self.cache_theta, self.cashe_r, 'ro', markersize=1)
        ax.set_ylim((0, 3))
        plt.get_current_fig_manager().set_window_title('Лидар')
        ax.set_yticks(np.arange(0, 3, 0.5))
        ax.set_xticks(np.linspace(0, 2*np.pi, 36, endpoint=False))
        ax.set_theta_offset(np.pi / 2)

        # Функция обновления графика
        def update_plot(_data):
            # минус так как лидар отсчитывает углы по часовой стрелке
            line.set_xdata(-self.theta) 
            line.set_ydata(self.r)
            return line, ax

        # Запуск потока для чтения данных с лидара
        Thread(target=self._readLoop, daemon=False).start()
        # Создание анимированного авто-обновляемого графика
        self.ani = animation.FuncAnimation(fig, update_plot,  interval=50)
        plt.show()

    # Метод для остановки измерений
    def stop(self):
        plt.close('all')
        self.is_run.set()

    # Приватный метод для чтения данных с лидара
    def _readLoop(self):
        ser = serial.Serial(
            self.config.port,
            self.config.baudrate
        )
        buffer = bytearray()
        # Установка частоты вращения лидара с помощью команды
        ser.write('LSRPM:{:d}H\n\r'.format(self.freq*60).encode('ascii'))

        # Кольцевой буфер для чтения пакетов байт с последовательного порта
        while not self.is_run.is_set():
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
            angles = np.linspace(angle/10, angle/10+36, length, endpoint=False)
            # Вычисление контрольной суммы для проверки целостности данных
            crc_calc = angle + length
            for i in range(length):
                crc_calc += data[i]
                # Расчет расстояния и интенсивности сигнала
                distances[i] = (data[i] & 0x1FFF) / 100 # Маскирование и масштабирование данных расстояния
                strengths[i] = (data[i] & 0xe000) >> 13 # Извлечение интенсивности сигнала
            if crc_in != crc_calc: # Проверка контрольной суммы
                print("error crc")
                return
            else: # Публикация данных сканирования, если данные корректны
                # Создание объекта LaserScan для публикации данных
                laser_scan = LaserScan()
                laser_scan.angle_min = math.radians(angles[0]) # Минимальный угол сканирования
                laser_scan.angle_max = math.radians(angles[-1]) # Максимальный угол сканирования
                laser_scan.angle_increment = math.radians(
                    angles[1] - angles[0]) # Шаг угла сканирования
                laser_scan.scan_time = 1.0 / self.freq # Время сканирования
                laser_scan.time_increment = 1.0 / self.freq / 10 / length # Временной интервал между измерениями
                laser_scan.range_min = 0. # Минимальное расстояние сканирования
                laser_scan.range_max = 30. # Максимальное расстояние сканирования
                laser_scan.ranges = distances.tolist() # Список измеренных расстояний
                laser_scan.intensities = strengths.tolist()  # Список интенсивностей
                self.lidar_.publish(laser_scan) # Публикация данных сканирования

                # Кэширование данных для отображения их на графике
                self.cache_theta = np.concatenate(
                    [self.cache_theta, np.radians(angles)])
                self.cashe_r = np.concatenate([self.cashe_r, distances])

                # Очистка кэша если прошло одно полное вращение
                if angle == 0:
                    self.theta = self.cache_theta
                    self.r = self.cashe_r
                    self.cache_theta = []
                    self.cashe_r = []


def main(args=None):
    rclpy.init(args=args)
    task = LidarNode(LidarConfig('/dev/ttyUSB0', 230400))

    def signal_handler(sig, frame):
        task.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)

    task.start()
    rclpy.spin(task)


if __name__ == '__main__':
    main()
