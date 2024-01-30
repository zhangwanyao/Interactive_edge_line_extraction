import open3d as o3d
import numpy as np
from scipy.spatial.distance import pdist, squareform
def intersection(cloud1,cloud2):
    """
    点云数据求交集
    len(cloud1)<len(cloud2)
    :param cloud1: point cloud
    :param cloud2: point cloud
    :return: intersection point cloud
    """
    # 构件KD树
    kdtree = o3d.geometry.KDTreeFlann(cloud2)
    # 半径搜索
    radius = 0.0001
    # 用于存储在cloud1中找到的与cloud2中每个点的最近邻索引
    indices = []
    # 遍历cloud1中的每个点
    for i in range(len(cloud1.points)):
        # 使用 KD 树搜索在半径radius内的最近邻点索引
        [_, idx, _] = kdtree.search_radius_vector_3d(cloud1.points[i], radius)
        idx = np.asarray(idx, dtype=int)  # 将索引转换为整数类型
        if len(idx) > 0:
            indices.append(i)
    # 提取点数据
    cloud3_points = [cloud1.points[i] for i in indices if i < len(cloud1.points)]
    cloud3 = o3d.geometry.PointCloud()
    cloud3.points = o3d.utility.Vector3dVector(cloud3_points)

    # Print Point Cloud Sizes
    # print(f"cloud1 has {len(cloud1.points)} points")
    # print(f"cloud2 has {len(cloud2.points)} points")
    # print(f"cloud3 has {len(cloud3.points)} points")

    # Save Point Cloud
    # o3d.visualization.draw_geometries([cloud3])
    # o3d.io.write_point_cloud("../cloud3.pcd", cloud3)
    return cloud3

def find_farthest_points(point_cloud):
    """
    求一组点云数据中欧式距离最远的两个点云数据
    :param point_cloud: pcd
    :return: points1，points2
    """
    point_cloud_points = np.asarray(point_cloud.points)
    # 计算任意两点之间的欧式距离
    distances = squareform(pdist(point_cloud_points))
    # distances = point_cloud.compute_nearest_neighbor_distance()
    max_distance_index = np.argmax(distances)

    point_index1, point_index2 = np.unravel_index(max_distance_index, distances.shape)

    # 提取距离最远的两个点的坐标
    point_1 = point_cloud_points[point_index1]
    point_2 = point_cloud_points[point_index2]

    # # 获取距离最远的两个点的索引
    # point_index1 = max_distance_index // len(point_cloud.points)
    # point_index2 = max_distance_index % len(point_cloud.points)
    #
    # # 提取距离最远的两个点的坐标
    # point_1 = point_cloud.points[point_index1]
    # point_2 = point_cloud.points[point_index2]

    return point_1, point_2