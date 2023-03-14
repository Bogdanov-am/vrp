import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import pygame
import threading
import time
from vrp_comp.util import ThrustMode, vector_thrust_decomposition

MODES = ['Direct-Mode', 'Vector-mode', 'H-mode', 'T-mode']

class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
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


        self.joyThread = threading.Thread(
            target=self.joystickThread, daemon=True).start()
        self.mode = ThrustMode.H_Mode

    def joystickThread(self):
        pygame.init()
        pygame.joystick.init()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 1:
                        if self.mode < 3:
                            self.mode +=1
                        else:
                            self.mode = 1
                        self.get_logger().info("Changed control mode. Current mode: {:s}".format(MODES[self.mode]))

            joystick_count = pygame.joystick.get_count()

            if joystick_count > 0:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()

                x1 = joystick.get_axis(0)
                y1 = -joystick.get_axis(1)

                x2 = joystick.get_axis(2)
                y2 = -joystick.get_axis(3)

                l, r, b = [.0, .0, .0]

                if self.mode == ThrustMode.Vector_Mode:
                    l, r, b = vector_thrust_decomposition(y1, -x1, x2)
                elif self.mode == ThrustMode.T_Mode:
                    l, r, b = [y1, -y1, x2]
                elif self.mode == ThrustMode.H_Mode:
                    l, r, b = [y1, -y2, .0]

                left = Float64()
                left.data = l
                right = Float64()
                right.data = r
                back = Float64()
                back.data = b

                self.right_.publish(right)
                self.left_.publish(left)
                self.back_.publish(back)

            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    manual = ManualControl()
    rclpy.spin(manual)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()
