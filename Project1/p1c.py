import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import signal
import numpy as np
from enum import Enum, auto

# Aus dem Bug Algorithmus Code

class Mode(Enum):
    FORWARD = auto()
    STOP = auto()
    TURN = auto()

class RoomMover(Node):
    def __init__(self):
        super().__init__('room_mover')

        self.pub_ = self.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)
        self.odom_subscr = self.create_subscription(Odometry, '/diff_drive_base_controller/odom', self.on_odom, 10)
        self.lidar_subscr = self.create_subscription(LaserScan, '/combined_laser_scan', self.on_lidar, 10)

        self.mode = Mode.FORWARD

        self.step_count = 0

    def on_odom(self, msg):
        pass

    def on_lidar(self, msg):
        pass