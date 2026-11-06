from enum import Enum, auto

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3  # https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html
from nav_msgs.msg import Odometry   # https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
from sensor_msgs.msg import Imu     # https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html

import csv

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
        self.odom_data = [['position.x', 'position.y', 'position.z',
                  'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w',
                  'linear.x', 'linear.y', 'linear.z',
                  'angular.x', 'angular.y', 'angular.z']]
        self.imu_data = [['orientation.x', 'orientation.y', 'orientation.z', 'orientation.w', 
                  'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z', 
                  'linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z']]
        
        self.velo_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.data_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(timer_period=MotionController.TIMER_PERIOD, callback=self.motion_sequence)
                         
        self.motion_sequence()

    def __stop_program(self):
        self.timer.cancel()
        rclpy.shutdown()
        self.save_data()

    def __change_motion(self, motion_phase: MotionPhase):
        twist_msg = Twist()
        linear_velocity = Vector3()
        angular_velocity = Vector3()

        if motion_phase == MotionPhase.FORWARD:
            linear_velocity.x = 0.5
        elif motion_phase == MotionPhase.TURN_LEFT:
            linear_velocity.x = 0.5
            angular_velocity.z = 0.5
        elif motion_phase == MotionPhase.TURN_RIGHT:
            linear_velocity.x = 0.5
            angular_velocity.z = -0.5
        elif motion_phase == MotionPhase.STOP:
            self.__stop_program()

        twist_msg.linear = linear_velocity
        twist_msg.angular = angular_velocity
        self.velo_pub.publish(twist_msg)

    def motion_sequence(self):

        if self.step_count == 0:
            motion_phase = MotionPhase.FORWARD
            self.get_logger().info('Motion started')
            self.__change_motion_and_log(motion_phase=motion_phase)
        
        elif self.step_count == 10:
            motion_phase = MotionPhase.TURN_LEFT
            self.__change_motion_and_log(motion_phase=motion_phase)

        elif self.step_count == 20:
            motion_phase = MotionPhase.TURN_RIGHT
            self.__change_motion_and_log(motion_phase=motion_phase)

        elif self.step_count == 30:
            motion_phase = MotionPhase.STOP
            self.__change_motion_and_log(motion_phase=motion_phase)
            self.get_logger().info('Motion stopped')
            
        self.step_count += 1

    def __change_motion_and_log(self, motion_phase: MotionPhase):
        self.__change_motion(motion_phase=motion_phase)
        self.get_logger().info(f'Tick {self.step_count} | New Motion Phase: {motion_phase.name}')

    def imu_callback(self, msg):
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        data = []

        data.append(orientation.x)
        data.append(orientation.y)
        data.append(orientation.z)
        data.append(orientation.w)

        data.append(angular_velocity.x)
        data.append(angular_velocity.y)
        data.append(angular_velocity.z)

        data.append(linear_acceleration.x)
        data.append(linear_acceleration.y)
        data.append(linear_acceleration.z)

        self.imu_data.append(data)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist

        data = []

        data.append(pose.position.x)
        data.append(pose.position.y)
        data.append(pose.position.z)
        data.append(pose.orientation.x)
        data.append(pose.orientation.y)
        data.append(pose.orientation.z)
        data.append(pose.orientation.w)

        data.append(twist.linear.x)
        data.append(twist.linear.y)
        data.append(twist.linear.z)
        data.append(twist.angular.x)
        data.append(twist.angular.y)
        data.append(twist.angular.z)

        self.odom_data.append(data)

    def save_data(self):
        folder = 'data'
        with open(f'{folder}/p1a_odom.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(self.odom_data)
        with open(f'{folder}/p1a_imu.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(self.imu_data)

def main():
    rclpy.init()
    motion_controller = MotionController()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()