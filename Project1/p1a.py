from enum import Enum, auto

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class MotionPhase(Enum):
    FORWARD = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    STOP = auto()

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        self.step_count = 0
        
        self.velo_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.data_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(timer_period=1, callback=self.motion_sequence)

        self.motion_phase = MotionPhase.FORWARD
        self.motion_changed = False
        self.__change_motion()

        self.get_logger().info('Motion started')

    def __change_motion(self):
        if not self.motion_changed:
            return

        if self.motion_phase == MotionPhase.FORWARD:
            pass
        elif self.motion_phase == MotionPhase.TURN_LEFT:
            pass
        elif self.motion_phase == MotionPhase.TURN_RIGHT:
            pass
        elif self.motion_phase == MotionPhase.STOP:
            pass

        self.motion_changed = False

    def motion_sequence(self):
        self.step_count += 1


        if self.step_count == 10:
            self.motion_phase = MotionPhase.TURN_LEFT
            self.motion_changed = True
        elif self.step_count == 20:
            self.motion_phase = MotionPhase.TURN_RIGHT
            self.motion_changed = True
        elif self.step_count == 30:
            self.motion_phase = MotionPhase.STOP
            self.motion_changed = True

        self.__change_motion()

    def imu_callback(self, msg):
        pass

    def odom_callback(self, msg):
        pass