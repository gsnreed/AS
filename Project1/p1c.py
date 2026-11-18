import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan # https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html
import numpy as np
from enum import Enum, auto
import random

# Aus dem Bug Algorithmus Code abgewandelt

class Mode(Enum):
    FORWARD = auto()
    TURN = auto()
    STOP = auto()

class RoomMover(Node):
    def __init__(self):
        super().__init__('room_mover')

        self.pub_ = self.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)
        self.odom_subscr = self.create_subscription(Odometry, '/diff_drive_base_controller/odom', self.on_odom, 10)
        self.lidar_subscr = self.create_subscription(LaserScan, '/combined_laser_scan', self.on_lidar, 10)

        self.mode = Mode.FORWARD

        self.theta_odom = None
        self.pos_odom = None
        self.ranges = None

        self.theta_target = None    # Sollwinkel

        # Konstanten
        self.timer_period = 0.5
        self.v_cmd = 0.2        # Sollgeschwindigkeit
        self.omega_cmd = 0.5    # Sollwinkelgeschwindigkeit
        self.min_collision_dist = 0.6   # Abstand zum Hindernis
        self.front_angle = 0.35 # Winkelbereich für Distanzmessung

        self.info = self.get_logger().info('Motion started')

        self.step_count = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def on_odom(self, msg):
        self.pos_odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.theta_odom = np.arcsin(msg.pose.pose.orientation.z) * 2.

    def on_lidar(self, msg):
        self.ranges = np.array(msg.ranges)
        self.ranges[np.isinf(self.ranges)] = 99.
        self.ranges[np.isnan(self.ranges)] = 99.
        self.phi_min, self.phi_increment = msg.angle_min, msg.angle_increment

    def timer_callback(self):
        if self.pos_odom is None or self.ranges is None:
            return
        self.sm()

    def get_front_distance(self):
        angles = np.arange(len(self.ranges)) * self.phi_increment + self.phi_min    # Winkel der Messungen (arange macht ein Array von 0 bis len(ranges)-1)
        front_indices = np.abs(angles) < self.front_angle   # Indizes der Messungen im Vorderbereich (True/False Array)
        front_ranges = self.ranges[front_indices]   # Messungen im Vorderbereich
        return np.min(front_ranges) # Minimale Distanz im Vorderbereich

    

def main(args=None):
    rclpy.init()
    motion_controller = RoomMover()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()

if __name__ == '__main__':
    main()