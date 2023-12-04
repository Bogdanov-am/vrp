import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import serial

# import socket
# ethRS485_converter_address = "192.168.0.7"
# ethRS485_converter_port = 26

PORT = '/dev/ttyUSB0'
adapter = serial.Serial(port=PORT, baudrate=9600, timeout=0.2, parity=serial.PARITY_NONE, \
                    stopbits=serial.STOPBITS_ONE, bytesize=8)


class ETHRS485(Node):
    def __init__(self):
        super().__init__('rs485')
        
        #For RS485 - USB converter
        self.rx_ = self.create_subscription(
            UInt8MultiArray,
            '/booblik/rs485Rx',
            self.request_callback,
            10
        )
        self.tx_ = self.create_publisher(
            UInt8MultiArray,
            '/booblik/rs485Tx',
            10
        )
        #For ETH - RS485 converter
        # self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.socket_.settimeout(1)
        # self.socket_.connect((ethRS485_converter_address, ethRS485_converter_port))
        # print("Успешное подключение к серверу")


    def request_callback(self, data:UInt8MultiArray):
        # try:
        #     print("Request:", list(data.data))
        #     self.socket_.sendall(data.data)
        #     answer = self.socket_.recv(1024)
        #     self.tx_answer(list(answer))

        #     print("Answer: ", list(answer))
        # except Exception as e:
        #     print("Request error {e}")
        #     print(e)

        try:
            print("Req: ", list(data.data))
            adapter.write(data.data)
            res = adapter.read(100)
            if len(list(res)) != 0:
                print("Res: ", list(res))
                self.tx_answer(list(res))
        except Exception as e:
            print("Request error")
            print(e)
    
    def tx_answer (self, data):
        try:
            msg = UInt8MultiArray()
            
            for i in data:
                msg.data.append(i)

            self.tx_.publish(msg)
        except Exception as e:
            print("Error send message")
            print(e)
            self.tx_.publish(UInt8MultiArray())#return empty


def main(args=None):
    rclpy.init(args=args)
    task = ETHRS485()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
