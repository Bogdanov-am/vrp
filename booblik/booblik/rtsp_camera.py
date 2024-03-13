
from threading import Thread

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image


class CameraConfig:
    """Информация для подключения к камере через rtsp"""
    login: str
    password: str
    ip: str
    port: str

class CameraNode(Node):
    def __init__(self, node_name: str, config: CameraConfig) -> None:
        super().__init__(node_name)
        self.config = config

        # Создание издателя для публикации данных с камеры
        self.camera_ = self.create_publisher(
            Image,
            '/booblik/sensors/cameras/camera/image_raw',
            10)
        self.cv_bridge = CvBridge()

    def start(self):
        # Запуск потока для чтения данных с камеры
        Thread(target=self._readLoop, daemon=False).start()

    # Приватный метод для чтения данных с камеры
    def _readLoop(self):

        link = f"rtspsrc location=rtsp://\
            {self.config.login}:{self.config.password}@{self.config.ip}:{self.config.port}\
            /ISAPI/Streaming/Channels/102 latency=50 ! decodebin ! videoconvert ! appsink"
        
        cap = cv2.VideoCapture(link, cv2.CAP_GSTREAMER)

        while True:
            # Получение кадра
            ok, img = cap.read()
            # Если кадр был получен, то публикуем его в топик
            if ok:
                self._push_image(img)
            else:
                print('Ошибка чтения')
                continue

    def _push_image(self, img: np.ndarray):
        ros2_img = self.cv_bridge.cv2_to_imgmsg(cvim=img, encoding='passthrough')
        self.camera_.publish(ros2_img)

def main(args=None):
    rclpy.init(args=args)
    cameraConfig = CameraConfig('admin', 'a123456789', '192.168.13.64', '554')
    task = CameraNode('camera', cameraConfig)
    task.start()
    rclpy.spin(task)
    rclpy.shutdown()