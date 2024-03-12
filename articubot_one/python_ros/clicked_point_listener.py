import rclpy
from geometry_msgs.msg import PointStamped

def clicked_point_callback(msg):
    print("Clicked Point: x={}, y={}, z={}".format(msg.point.x, msg.point.y, msg.point.z))

def main(args=None):    
    rclpy.init(args=args)
    node = rclpy.create_node('clicked_point_listener')
    subscriber = node.create_subscription(PointStamped, '/clicked_point', clicked_point_callback)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
