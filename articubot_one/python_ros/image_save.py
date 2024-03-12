import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 여기에 실제 사용하는 카메라 토픽 이름을 입력하세요
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning

        self.cv_bridge = CvBridge()
        self.image_count = 0

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("Error converting image: %s" % str(e))
            return

        # 이미지를 저장하거나 처리할 수 있습니다.
        image_filename = "image_%05d.png" % self.image_count
        cv2.imwrite(image_filename, cv_image)
        self.image_count += 1
        self.get_logger().info("Saved image: %s" % image_filename)

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
