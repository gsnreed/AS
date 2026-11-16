import rclpy
from rclpy.node import Node

class RoomMover(Node):
    def __init__(self):
        super().__init__('room_mover')

        self.step_count = 0

        
