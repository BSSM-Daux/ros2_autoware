import open3d as o3d

# .pcd 파일 불러오기
point_cloud = o3d.io.read_point_cloud("point_cloud.pcd")

# 시각화
o3d.visualization.draw_geometries([point_cloud])
