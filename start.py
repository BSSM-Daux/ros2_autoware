import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('move_to_goal')

    goal_publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)

    # 원하는 위치로 이동하기 위한 목표 좌표 설정
    # x=0.010886, y=0.000070, z=0.050000
    # x=-0.165938, y=0.152139, z=0.049998
    #  x=-2.430519, y=0.296485, z=0.050000
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = -2.430519
    goal_pose.pose.position.y = 0.296485
    goal_pose.pose.position.z = 0.050000
    goal_pose.pose.orientation.w = 1.0 # 기본 방향으로 설정

    while goal_publisher.get_subscription_count() < 1:
        rclpy.spin_once(node)

    # 목표 좌표를 게시하여 로봇을 이동시킴
    goal_publisher.publish(goal_pose)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
