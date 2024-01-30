import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import pairwise_distances
from Curvature_calculation import caculate_surface_curvature
from intersection import intersection
from intersection import find_farthest_points
class PointCloudProcessing:
    def __init__(self, point_cloud_path):
        self.point_cloud = o3d.io.read_point_cloud(point_cloud_path)
        self.seed_point = None
        self.picked_indices = None
    def pick_points(self):
        # 交互式选择点
        print("")
        print(
            "1) 请至少选择三个对应点，使用 [Shift + 左键单击]"
        )
        print("   按 [Shift + 右键单击] 取消点的选择")
        print("2) 选择完点后，按 'Q' 键关闭窗口")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(self.point_cloud)
        vis.run()  # 用户选点
        vis.destroy_window()
        print("")
        picked_indices = np.asarray(vis.get_picked_points())
        if len(picked_indices) > 0:
            # 将列表转为数组
            self.picked_indices = np.asarray(picked_indices)
            self.point_cloud.colors[self.picked_indices] = [1, 0, 0]
            self.seed_point = np.array(self.point_cloud.points)[picked_indices]
            print(self.seed_point)
        else:
            print("No points picked.")


    def extract_local_points(self,search_radius):
        # 提取半径范围内点云数据
        [k, idx, _] = pcd_tree.search_radius_vector_3d(self.point_cloud.points[self.picked_indices],search_radius)
        np.asarray(self.point_cloud.colors)[idx[1:], :] = [0, 1, 0]
        print("Visualize the point cloud.")
        o3d.visualization.draw_geometries([self.point_cloud])
        local_points = np.asarray(self.point_cloud.points)[idx[1:], :]
        local_points = np.append(local_points, self.seed_point,axis=0)
        local_points_pc = o3d.geometry.PointCloud()
        local_points_pc.points = o3d.utility.Vector3dVector(local_points)
        # o3d.visualization.draw_geometries([local_points_pc])
        return local_points_pc


    def fit_planes(self, pcd):
        # 拟合平面求两平面相距最近的点云
        segment_models = {}  # 用于存储平面分割模型的字典
        segments = {}  # 存储分割出的点云片段的字典
        max_plane_idx = 2  # 最大平面数量
        rest = pcd  # 初始点云，用于迭代分割
        d_threshold = 0.01  # DBSCAN聚类时的距离阈值

        # 遍历每个平面进行分割
        for i in range(max_plane_idx):
            # 使用RANSAC算法进行平面分割
            colors = plt.get_cmap("tab20")(i)
            segment_models[i], inliers = rest.segment_plane(distance_threshold=2, ransac_n=5, num_iterations=1000)
            # 选择平面内的点云
            segments[i] = rest.select_by_index(inliers)

            # 使用DBSCAN进行聚类，获取每个聚类的标签
            labels = np.array(segments[i].cluster_dbscan(eps=d_threshold * 10, min_points=10))
            # 计算每个聚类的点数
            candidates = [len(np.where(labels == j)[0]) for j in np.unique(labels)]
            # 选择点数最多的聚类作为最好的拟合面
            best_candidate = int(np.unique(labels)[np.where(candidates == np.max(candidates))[0]])

            # print("最好的拟合面是：", best_candidate)

            # 更新剩余点云，去除当前平面内的点和非最好拟合面的点
            rest = rest.select_by_index(inliers, invert=True) + segments[i].select_by_index(
                list(np.where(labels != best_candidate)[0]))

            # 更新当前平面为最好拟合面的点
            segments[i] = segments[i].select_by_index(list(np.where(labels == best_candidate)[0]))
            segments[i].paint_uniform_color(list(colors[:3]))

            # print("pass", i + 1, "/", max_plane_idx, "done.")
        # 可视化平面分割效果
        # o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)])

        # 将 Open3D 的点云数据转换为 NumPy 数组
        points_i = np.asarray(segments[0].points)
        points_j = np.asarray(segments[1].points)

        # 计算欧式距离矩阵
        distances_matrix = pairwise_distances(points_i, points_j)

        # 找到距离小于10的点的索引
        row_indices, col_indices = np.where(distances_matrix < 10)
        # 将满足条件的点添加到最近点列表
        closest_points = np.concatenate([points_i[row_indices], points_j[col_indices]])
        # 将 closest_points 转换为 Open3D 点云对象
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(closest_points)
        # o3d.io.write_point_cloud("../point_cloud.pcd", point_cloud)
        return point_cloud

    def extract_local_points2(self,search_radius):
        """
        迭代寻找近邻点，用于后续更新
        :param search_radius: 搜索半径
        :return: 半径点云
        """
        print(f"seed_point{self.seed_point}")
        [k, idx, _] = pcd_tree.search_radius_vector_3d(self.seed_point,search_radius)
        # o3d.visualization.draw_geometries([self.point_cloud])
        local_points = np.asarray(self.point_cloud.points)[idx[1:], :]
        local_points = np.vstack([local_points, self.seed_point])
        local_points_pc = o3d.geometry.PointCloud()
        local_points_pc.points = o3d.utility.Vector3dVector(local_points)
        # o3d.visualization.draw_geometries([local_points_pc])
        return local_points_pc
    def process_iteration(self, num_iterations,intersection_cloud_list):
        # 迭代细化生长过程
        merged_cloud = o3d.geometry.PointCloud()
        for iteration in range(num_iterations):
            print(f"Iteration {iteration + 1} / {num_iterations}")

            # 设置种子点
            # 求交集中欧式距离最远的两点
            point_1, point_2 = find_farthest_points(intersection_cloud_list[-1])
            points = [point_1, point_2]
            print(f"points{points}")
            and_cloud = o3d.geometry.PointCloud()
            for point in points:
                self.seed_point = np.array(point)
                # 2. 提取感兴趣范围内点云数据
                search_radius = 200  # 设置搜索半径
                local_pcd = self.extract_local_points2(search_radius)
                # o3d.io.write_point_cloud(f"../local_pcd_iteration_{iteration}.pcd", local_pcd)

                # 3. 对感兴趣的区域点进行曲率计算
                surface_curvature = caculate_surface_curvature(local_pcd, radius=5)

                # # 4. 排序并绘制曲率分布直方图
                # sorted_curvature = np.sort(surface_curvature)
                # n, bins, patches = plt.hist(sorted_curvature, bins=100, edgecolor='black')
                # plt.title('Surface Curvature Distribution')
                # plt.xlabel('Curvature')
                # plt.ylabel('Frequency')
                # plt.show()

                # 5. 曲率阈值输入
                # threshold = float(input("请输入曲率阈值："))

                # 6. 曲率筛选点云数据
                point_array = np.asarray(local_pcd.points)
                interesting_points = point_array[surface_curvature > threshold]

                # 7. 保存提取后的点云数据
                interesting_pcd = o3d.geometry.PointCloud()
                interesting_pcd.points = o3d.utility.Vector3dVector(interesting_points)
                # o3d.io.write_point_cloud(f"../interesting_pcd_iteration_{iteration}.pcd", interesting_pcd)

                # 8. 平面分割
                plane_cloud = self.fit_planes(local_pcd)

                # 9. 求分割点云和局部曲率较大值点云的交集
                intersection_cloud = intersection(interesting_pcd, plane_cloud)
                and_cloud += intersection_cloud
                # 将当前的 intersection_cloud 合并到 merged_cloud 中
                merged_cloud += intersection_cloud

                # 将 merged_cloud 的颜色修改为红色
                red_color = [1, 0, 0]  # RGB颜色
                merged_cloud.paint_uniform_color(red_color)

            intersection_cloud_list.append(and_cloud)
        return merged_cloud

if __name__ == "__main__":
    point_cloud_path = r'C:\Users\zhaojunzhe\Desktop\pointcloud\bottom_staircase.pcd'
    # 声明类
    processor = PointCloudProcessing(point_cloud_path)
    # 按shift+左键选取感兴趣点
    processor.pick_points()
    search_radius = 50 #设置搜索半径
    pcd_tree = o3d.geometry.KDTreeFlann(processor.point_cloud)
    # 提取感兴趣范围内点云数据
    local_pcd = processor.extract_local_points(search_radius)
    # o3d.io.write_point_cloud("../local_pcd.pcd", local_pcd)
    # 对感兴趣的区域点进行曲率计算
    surface_curvature = caculate_surface_curvature(local_pcd, radius=5)
    # 排序
    sorted_curvature = np.sort(surface_curvature)
    # 绘制直方图，并显示每个柱子的数量
    n, bins, patches = plt.hist(sorted_curvature, bins=100, edgecolor='black')
    plt.title('Surface Curvature Distribution')
    plt.xlabel('Curvature')
    plt.ylabel('Frequency')

    # 显示每个柱子的数量
    for i in range(len(patches)):
        plt.text(patches[i].get_x() + patches[i].get_width() / 2, patches[i].get_height(),
                 f'{int(n[i])}', ha='center', va='bottom')

    # 显示每个柱子的中心x坐标
    for i in range(len(bins) - 1):
        plt.text((bins[i] + bins[i + 1]) / 2, 0, f'{int(n[i])}', ha='center', va='bottom')

    plt.show()
    # 按照曲率显示点云
    threshold = float(input("请输入曲率阈值："))
    # 将点云数据转换为NumPy数组
    point_array = np.asarray(local_pcd.points)

    # 按照曲率筛选点云数据
    interesting_points = point_array[surface_curvature > threshold]

    # 保存提取后的点云数据
    interesting_pcd = o3d.geometry.PointCloud()
    interesting_pcd.points = o3d.utility.Vector3dVector(interesting_points)
    # o3d.io.write_point_cloud("../interesting_pcd.pcd", interesting_pcd)
    # 平面分割
    plane_cloud = processor.fit_planes(local_pcd)

    # 求分割点云和局部曲率较大值点云的交集
    intersection_cloud = intersection(interesting_pcd,plane_cloud)
    # 设置迭代次数
    num_iterations = 100
    # 在调用时创建一个列表
    intersection_cloud_list = []
    intersection_cloud_list.append(intersection_cloud)
    merged_point_cloud = processor.process_iteration(num_iterations,intersection_cloud_list)
    red_color = np.array([[1.0, 0.0, 0.0]])
    merged_point_cloud.colors = o3d.utility.Vector3dVector(np.tile(red_color, (len(merged_point_cloud.points), 1)))
    o3d.visualization.draw_geometries([merged_point_cloud, processor.point_cloud])
    # 保存合并后的点云
    o3d.io.write_point_cloud("../merged_point_cloud.pcd", merged_point_cloud)