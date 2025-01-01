import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class Block_Node(Node):
    def __init__(self):
        super().__init__('block_node')
        self.ir_subscription = self.create_subscription(Image, '/my_camera/ir/image_raw', self.ir_image_callback, 10)
        self.depth_subscription = self.create_subscription(Image, '/my_camera/depth/image_raw', self.depth_image_callback, 10)
        self.coordinate_publisher = self.create_publisher(Float32MultiArray, '/my_camera/block_coordinates', 10)
        
        self.bridge = CvBridge()
        self.ir_image = None
        self.depth_image = None

    def ir_image_callback(self, msg):
        self.ir_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        self.ir_image = cv2.normalize(self.ir_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        self.detect_box()

    def detect_box(self):
        if self.ir_image is None or self.depth_image is None:
            return
        
        # Set ROI 
        height, width = self.ir_image.shape
        roi_height = int(height / 6)
        roi_width = int(width/2)
        roi = self.ir_image[roi_height:(5*roi_height), :(roi_width)]
        cv2.rectangle(self.ir_image, (0, roi_height), (roi_width, (5*roi_height)), (0, 255, 0), 3)

        # IR process, Countour
        _, binary_ir_image = cv2.threshold(roi, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary_ir_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        block_coordinates = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if 4000 > w * h > 500:  # Range Pixel
                cx, cy = x + w // 2, y + h // 2  
                z_value = self.depth_image[(cy + roi_height), cx]
          
                block_coordinates.extend([cx, cy, z_value])
                cv2.rectangle(self.ir_image, (x, y + roi_height), (x + w, y + h + roi_height), (255, 255, 0), 2)

        self.publish_coordinates(block_coordinates)

    def publish_coordinates(self, block_coordinates):
        if block_coordinates:
            msg = Float32MultiArray() 
            msg.data.extend(block_coordinates)
            self.coordinate_publisher.publish(msg)
            self.get_logger().info(f"Block_Coordinates(x,y,z): {block_coordinates}")

        cv2.imshow("IR CAM", self.ir_image)
        cv2.imshow("Depth CAM", self.depth_image / self.depth_image.max() * 255)  
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Block_Node()
    rclpy.spin(node)
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()