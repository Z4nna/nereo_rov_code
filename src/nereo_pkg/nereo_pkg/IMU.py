import numpy as np
import rclpy
from rclpy.node import Node

'''
Leggere documentazione imu, che deve pubblicare pitch e roll
'''

class imuNode(Node):

    def __init__(self):
        super().__init__("IMU_node")
        self.get_logger().info("IMU node connected")
    
    def publish_values(self):
        values = 
