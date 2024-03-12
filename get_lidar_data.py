import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import open3d as o3d
import numpy as np

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

    def scan_callback(self, msg):
        self.get_logger().info('Received Scan Message:')
        self.get_logger().info('Ranges: {}'.format(msg.ranges))
        self.get_logger().info('Intensities: {}'.format(msg.intensities))
        
        # 임시 배열 생성
        points = []
        
        # 레이저 스캔 데이터 처리
        for i, distance in enumerate(msg.ranges):
            intensity = msg.intensities[i]

            # 레이저 스캐너의 위치 및 방향
            scanner_x = 0
            scanner_y = 0
            scanner_z = 1
            angle_vertical = msg.angle_min + i * msg.angle_increment
            angle_horizontal = 0  # 예시로 0으로 설정

            # 레이저 스캐너의 회전 반경
            radius = distance * math.sin(angle_vertical)

            # x, y, z 좌표 계산
            x = scanner_x + radius * math.cos(angle_horizontal)
            y = scanner_y + radius * math.sin(angle_horizontal)
            z = scanner_z + distance * math.cos(angle_vertical)

            # 포인트 추가
            points.append([x, y, z])

        # numpy 배열로 변환
        points = np.array(points)
        
        # open3d의 PointCloud 객체 생성
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)

        # pcd 파일로 저장
        o3d.io.write_point_cloud("point_cloud.pcd", point_cloud)
        self.get_logger().info('Point cloud saved as point_cloud.pcd')

def main(args=None):
    rclpy.init(args=args)
    subscriber = ScanSubscriber()
    rclpy.spin(subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
