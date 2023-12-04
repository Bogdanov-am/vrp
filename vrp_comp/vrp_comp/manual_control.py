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

        pygame.init()
        pygame.joystick.init()

        self.joyThread = threading.Thread(
            target=self.joystickThread, daemon=True).start()
        self.mode = ThrustMode.H_Mode

        self.keyboardThread = threading.Thread(
            target=self.keyboardControlThread, daemon=True).start()


    def joystickThread(self):
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
                left.data = l * 20.0
                right = Float64()
                right.data = r * 20.0
                back = Float64()
                back.data = b * 20.0

                self.right_.publish(right)
                self.left_.publish(left)
                self.back_.publish(back)

            time.sleep(0.1)


    def keyboardControlThread(self):
        def update_axis(axis, shift, max):
            if(axis > 0.0):
                axis = axis - shift
            elif(axis < 0.0):
                axis = axis + shift

            if(axis > max):
                axis = max
            elif(axis < -max):
                axis = -max

            return axis
            
        def shift_axis(axis, shift, max):
            axis = axis + shift
            if(axis > max):
                axis = max
            elif(axis < -max):
                axis = -max
            
            return axis


        x_axis_emulate = 0.0
        y_axis_emulate = 0.0
        axis_max_value = 100.0
        joystick_count = pygame.joystick.get_count()
        screen = None
        clock = None

        white_color = (255,255, 255)
        green_color = (0, 255, 0)
        blue_color = (0, 0, 255)
        red_color = (255, 0, 0)

        FPS = 60
        SCREEN_X_SIZE = 400
        SCREEN_Y_SIZE = 400
        if (joystick_count == 0):
            screen = pygame.display.set_mode((SCREEN_X_SIZE, SCREEN_Y_SIZE))
            clock = pygame.time.Clock()
            pygame.display.set_caption("Keyboard control")
        else:
            exit()
        while(1):
            shift = 1.0

            #back to 0
            x_axis_emulate = update_axis(x_axis_emulate, shift, axis_max_value)
            y_axis_emulate = update_axis(y_axis_emulate, shift, axis_max_value)

            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                x_axis_emulate = shift_axis(x_axis_emulate, -shift * 2, axis_max_value)
            elif keys[pygame.K_RIGHT]:
                x_axis_emulate = shift_axis(x_axis_emulate, shift * 2, axis_max_value)
            if keys[pygame.K_UP]:
                y_axis_emulate = shift_axis(y_axis_emulate, shift * 2, axis_max_value)
            elif keys[pygame.K_DOWN]:
                y_axis_emulate = shift_axis(y_axis_emulate, -shift * 2, axis_max_value)
            
            screen.fill(white_color)


            x_emulate_shift = (x_axis_emulate / axis_max_value) * (SCREEN_X_SIZE / 2)
            y_emulate_shift = (y_axis_emulate / axis_max_value) * (SCREEN_X_SIZE / 2)

            #draw lines
            pygame.draw.line(screen, (0,0,0), (0, SCREEN_X_SIZE / 2), (SCREEN_X_SIZE, SCREEN_X_SIZE / 2))
            pygame.draw.line(screen, (0,0,0), (SCREEN_X_SIZE / 2, 0), (SCREEN_X_SIZE / 2, SCREEN_X_SIZE))

            #draw axis value
            pygame.draw.circle(screen, red_color, 
                            (SCREEN_X_SIZE / 2 + x_emulate_shift,
                              SCREEN_Y_SIZE / 2), 8)
            pygame.draw.circle(screen, blue_color, 
                            (SCREEN_X_SIZE / 2,
                              SCREEN_Y_SIZE / 2 - y_emulate_shift), 7)
            

            l, r, b = [y_axis_emulate / axis_max_value, 
                       -(y_axis_emulate / axis_max_value), 
                       x_axis_emulate / axis_max_value]
            
            left = Float64()
            left.data = l * 20.0
            right = Float64()
            right.data = r * 20.0
            back = Float64()
            back.data = b * 20.0
            if (pygame.joystick.get_count() == 0):
                self.right_.publish(right)
                self.left_.publish(left)
                self.back_.publish(back)

            pygame.display.update()
            clock.tick(FPS)


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
