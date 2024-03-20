import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class WheelPublisher(Node):
    def __init__(self):
        super().__init__('wheel_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        
        timer_period = 0.1 # 반복할 시간을 선택한다
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.i = 0.0
        self.i2 = 0.0
        
        self.a = 0.0
        self.b = 0.0

        # 압뒤 좌우 값
        self.x = 0.0
        self.y = 0.0
        self.theta = 3.14 # 회전 값
        
    def timer_callback(self):
        self.i += self.a    # left 바퀴
        self.i2 += self.b   # right 바퀴
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint']
        msg.position = [self.i] 
        
        msg2 = JointState()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.name = ['right_wheel_joint']
        msg2.position = [self.i2]

        

        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.position[0])
        self.publisher_.publish(msg2)
        #self.get_logger().info('Publishing: "%s"' % msg2.position[0])

    def listener_callback(self, msg):
        # 속도및 움직임 값을 받아 온다
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        right_wheel_velocity = linear_velocity + angular_velocity
        left_wheel_velocity = linear_velocity - angular_velocity

        # JointState 메시지 생성
        right_wheel_state = JointState()
        right_wheel_state.name = ['right_wheel_joint']
        right_wheel_state.velocity = [right_wheel_velocity]

        left_wheel_state = JointState()
        left_wheel_state.name = ['left_wheel_joint']
        left_wheel_state.velocity = [left_wheel_velocity]
        print(left_wheel_state.velocity[0])
        print(right_wheel_state.velocity[0])

        self.a = left_wheel_state.velocity[0]
        self.b = right_wheel_state.velocity[0]


def main(args=None):
    rclpy.init(args=args)

    wheel_publisher = WheelPublisher()

    rclpy.spin(wheel_publisher)

    wheel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # ser = serial.Serial('/dev/ttyACM0', 112500, timeout=1)

    main()
