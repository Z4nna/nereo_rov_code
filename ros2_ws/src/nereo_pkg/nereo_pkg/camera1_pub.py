import rclpy
from rclpy.node import Node
import sensor_msgs.msg

import cv2
from cv_bridge import CvBridge

def main(args = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_1_publisher')

        self.declare_parameters(
            namespace='',
            parameters= [
                ('fps', 30),
                ('size', (720,480)),
                ('flip_horizontal', False),
                ('flip_vertical', False)
            ]
        )
        self.camera = cv2.VideoCapture(0)  #('/home/ubuntu/Videos/video.mp4')
        self.publisher = self.create_publisher(sensor_msgs.msg.Image, '/camera1', 5)
        self.get_logger().info('Camera node running')
        self.send_frame
        self.bridge = CvBridge()
        ## sets a timer with the requested fps
        self.create_timer(int(1000/self.get_parameter('fps')._value), self.send_frame)
    
    def send_frame(self):
        if not self.camera.isOpened(): return
        ret, frame = self.camera.read()
        if not ret: return
        ## eventually flips the image, as requested by the parameters
        flip_horizontal = self.get_parameter('flip_horizontal')._value
        flip_vertical = self.get_parameter('flip_vertical')._value

        # Check data types
        self.get_logger().info(f'Type of flipH: {type(flip_horizontal)}\nType of flipV: {type(flip_vertical)}\n')

        if flip_horizontal and flip_vertical:
            frame = cv2.flip(frame, -1)
        elif flip_horizontal:
            frame = cv2.flip(frame, 1)
        elif flip_vertical:
            frame = cv2.flip(frame, 0)

        frame = cv2.resize(frame, self.get_parameter('size')._value, interpolation=cv2.INTER_AREA)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

if __name__ == "__main__":
    main()