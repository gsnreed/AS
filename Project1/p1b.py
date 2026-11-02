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
        self.g = np.array([0., 0., 9.807])         # world system with z axis up assumed
        self.first = True
        self.last_t = 0.
        self.t0 = 0.
        self.subscription = ...
        self.pub_ = ...
        
    def quat2DCM(self, q):
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

    def quatDEQ(self, w):
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
        
        ...
        
        self.get_logger().info(f't: {t-self.t0:.3f}: pos {self.p_w}, vel {self.v_w}, a {a_w_mes}, quat {self.q_bw}')
        msg = Odometry()
        
        ...
        
        self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(INA())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
