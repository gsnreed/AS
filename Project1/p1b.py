import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3
import numpy as np

class INA(Node):

    def __init__(self):
        super().__init__('inertial_navigation')
        self.p_w = np.array([0., 0., 0.])
        self.v_w = np.array([0., 0., 0.])
        self.C_wb = np.identity(3)
        self.q_bw = np.array([1., 0., 0., 0.])     # wxyz 
        self.g = np.array([0., 0., 9.807])         # world system with z axis up assumed (Gravitation)
        self.first = True
        self.last_t = 0.
        self.t0 = 0.
        self.subscription = self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.pub_ = self.create_publisher(Odometry, '/inertial_navigation/odom', 10)
        
    def quat2DCM(self, q):  # AS3 S. 46
        """Quaternion in Rotationsmatrix umwandeln"""
        q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 + q0 * q3)
        r02 = 2 * (q1 * q3 - q0 * q2)
        r10 = 2 * (q1 * q2 - q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 + q0 * q1)
        r20 = 2 * (q1 * q3 + q0 * q2)
        r21 = 2 * (q2 * q3 - q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        return np.array([[r00, r01, r02],
                         [r10, r11, r12],
                         [r20, r21, r22]])

    def quatDEQ(self, w):   # AS3 S. 46
        """Quaternion differential equation"""
        wx, wy, wz = w[0], w[1], w[2]
        return 0.5 * np.array([[0., -wx, -wy, -wz],
                               [wx,  0.,  wz, -wy],
                               [wy, -wz,  0.,  wx],
                               [wz,  wy, -wx,  0.]])

    def on_imu(self, msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.first:
            self.last_t = t
            self.t0 = t
            self.first = False
            return
        dt = t - self.last_t
        self.last_t = t

        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        w = np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z])  # Winkelgeschwindigkeitsvektor
        a_b = np.array([linear_acceleration.x, linear_acceleration.y, linear_acceleration.z])  # gemessene Beschleunigung im Körperkoordinatensystem

        q_dot = self.quatDEQ(w) @ self.q_bw # Änderungsrate der Orientierung berechnen (q_dot = A(w) * q_bw)
        self.q_bw += q_dot * dt

        self.q_bw /= np.linalg.norm(self.q_bw)  # Normalisierung

        self.C_wb = self.quat2DCM(self.q_bw).T  # Rotationsmatrix aus Quaternion berechnen

        a_w_mes = self.C_wb @ a_b - self.g   # Beschleunigung im Weltkoordinatensystem (a_w = C_wb * a_w_mess + g)

        self.p_w += self.v_w * dt + 0.5 * a_w_mes * dt**2   # Position
        self.v_w += a_w_mes * dt    # Geschwindigkeit
        
        self.get_logger().info(f't: {t-self.t0:.3f}: pos {self.p_w}, vel {self.v_w}, a {a_w_mes}, quat {self.q_bw}')
        msg = Odometry()    # https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.p_w[0]
        msg.pose.pose.position.y = self.p_w[1]
        msg.pose.pose.position.z = self.p_w[2]

        msg.pose.pose.orientation.x = self.q_bw[1]
        msg.pose.pose.orientation.y = self.q_bw[2]
        msg.pose.pose.orientation.z = self.q_bw[3]
        msg.pose.pose.orientation.w = self.q_bw[0]

        msg.twist.twist.linear.x = self.v_w[0]
        msg.twist.twist.linear.y = self.v_w[1]
        msg.twist.twist.linear.z = self.v_w[2]

        msg.twist.twist.angular.x = w[0]
        msg.twist.twist.angular.y = w[1]
        msg.twist.twist.angular.z = w[2]
        
        self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(INA())
    rclpy.shutdown()

if __name__ == '__main__':
    main()