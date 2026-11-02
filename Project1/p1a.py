from enum import Enum, auto

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3  # https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
from nav_msgs.msg import Odometry   # https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
from sensor_msgs.msg import Imu     # https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html

class MotionPhase(Enum):
    FORWARD = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    STOP = auto()

class MotionController(Node):
    TIMER_PERIOD = 1.0  # seconds

    def __init__(self):
        super().__init__('motion_controller')

        self.step_count = 0
        
        self.velo_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.data_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.motion_phase = MotionPhase.FORWARD
        self.motion_changed = True
        self.timer = self.create_timer(timer_period=self.TIMER_PERIOD, callback=self.motion_sequence)
        self.__change_motion()

        self.get_logger().info('Motion started')

    def __stop_program(self):
        self.get_logger().info('Stopping motion')
        self.timer.cancel()
        rclpy.shutdown()

    def __change_motion(self):
        if not self.motion_changed:
            return

        self.get_logger().info(f'Tick {self.step_count} | New Motion Phase: {self.motion_phase.name}')

        twist_msg = Twist()
        linear_velocity = Vector3()
        angular_velocity = Vector3()

        if self.motion_phase == MotionPhase.FORWARD:
            linear_velocity.x = 0.5
        elif self.motion_phase == MotionPhase.TURN_LEFT:
            linear_velocity.x = 0.5
            angular_velocity.z = 0.5
        elif self.motion_phase == MotionPhase.TURN_RIGHT:
            linear_velocity.x = 0.5
            angular_velocity.z = -0.5
        elif self.motion_phase == MotionPhase.STOP:
            self.__stop_program()

        twist_msg.linear = linear_velocity
        twist_msg.angular = angular_velocity
        self.velo_pub.publish(twist_msg)

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

def main():
    rclpy.init()
    motion_controller = MotionController()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()