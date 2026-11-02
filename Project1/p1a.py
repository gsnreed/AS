import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        self.step_count = 0
        
        self.velo_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.data_sub = self.create_subscription(Imu, '/cmd_vel', self.data_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(timer_period=0.1, callback=self.timer_callback)

        self.get_logger().info('Motion started')

    def timer_callback(self):
        pass

    def data_callback(self, msg):
        pass

    def odom_callback(self, msg):
        pass