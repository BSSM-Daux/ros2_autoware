import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPoseSubscriber(Node):
    def __init__(self):
        super().__init__('goal_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        self.subscription

    def goal_pose_callback(self, msg):
        self.get_logger().info('Received Goal Pose Message:')
        self.get_logger().info('Position: x={}, y={}, z={}'.format(
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        self.get_logger().info('Orientation: x={}, y={}, z={}, w={}'.format(
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

def main(args=None):
    rclpy.init(args=args)
    goal_pose_subscriber = GoalPoseSubscriber()
    rclpy.spin(goal_pose_subscriber)
    goal_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
