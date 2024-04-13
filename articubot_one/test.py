import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class WheelMonitor(Node):

    def __init__(self):
        super().__init__('wheel_monitor')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription # 사용되지 않는 변수 경고를 방지하기 위해

    def listener_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'left_wheel_joint':
                self.get_logger().info(f'왼쪽 바퀴 위치: {msg.position[i]}')

    def publish_joint_states(self, position):
        msg = JointState()
        msg.name = ['left_wheel_joint']
        msg.position = [position]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    wheel_monitor = WheelMonitor()

    # 예시로 왼쪽 바퀴의 위치를 0.5로 설정하여 게시
    wheel_monitor.publish_joint_states(0.5)

    rclpy.spin(wheel_monitor)

    wheel_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

