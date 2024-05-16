import numpy as np
from scipy.spatial import KDTree
from plyfile import PlyData, PlyElement
from sklearn.linear_model import RANSACRegressor
#读取点云文件函数
def load_point_cloud(filename):
    """Load point cloud data from a PLY file."""
    plydata = PlyData.read(filename)
    points = np.vstack([plydata['vertex']['x'], plydata['vertex']['y'], plydata['vertex']['z']]).T
    return points

# def iterative_closest_points(target_points, source_points, max_iterations=200, tolerance=1e-6, dmax=1):
#     """Perform Iterative Closest Point (ICP) algorithm."""
#     transformation = np.identity(4)
#     trans = np.identity(4)
    
#     for _ in range(max_iterations):
#         trans_points = apply_transformation(source_points.T, trans).T
#         neighbor = find_correspondences(trans_points, target_points)
        
#         weights = np.where(np.linalg.norm(neighbor - trans_points, axis=1) <= dmax, 1, 0)
#         non_zero_indices = np.nonzero(weights)[0]
        
#         sources = trans_points[non_zero_indices]
#         targets = neighbor[non_zero_indices]
        
#         transformation = solve_least_squares(sources, targets)
#         trans = np.dot(transformation, trans)
        
#         if np.linalg.norm(transformation - np.identity(4)) < tolerance:
#             break
    
#     inverse_matrix = np.linalg.inv(trans)
#     results_points = apply_transformation(target_points.T, inverse_matrix).T
    
#     return results_points, trans

#ICP函数
def iterative_closest_points(target_points, source_points, max_iterations=200, error_tolerance=1e-6, dmax=1):
    """Perform Iterative Closest Point (ICP) algorithm."""
    transformation = np.identity(4)
    trans = np.identity(4)
    #采用连续两次变换矩阵的误差作为收敛判定条件
    prev_transformation = np.identity(4)
    
    for _ in range(max_iterations):
        #对源点云进行变换，并在目标点云中进行最近邻搜索
        trans_points = apply_transformation(source_points.T, trans).T
        neighbor = find_correspondences(trans_points, target_points)
        #判断最近邻的距离是否满足《=dmax，并赋予权重
        weights = np.where(np.linalg.norm(neighbor - trans_points, axis=1) <= dmax, 1, 0)
        non_zero_indices = np.nonzero(weights)[0]
        
        sources = trans_points[non_zero_indices]
        targets = neighbor[non_zero_indices]
        #提取权重为1的点对并进行最小二乘问题求解
        transformation = solve_least_squares(sources, targets)
        trans = np.dot(transformation, trans)
        #判断是否收敛
        error = np.linalg.norm(transformation - prev_transformation)
        if error < error_tolerance:
            inverse_matrix = np.linalg.inv(trans)
            results_points = apply_transformation(source_points.T, trans).T
    
            print('iterate finished')
            break
        
        prev_transformation = transformation
    #输出变换后的点云
    inverse_matrix = np.linalg.inv(trans)
    results_points = apply_transformation(source_points.T, trans).T
    
    return results_points, trans
#最小二乘函数
def solve_least_squares(source_points_nearest, target_points_nearest):
    """Solve least squares problem for point cloud registration."""
    source_center = np.mean(source_points_nearest, axis=0)
    target_center = np.mean(target_points_nearest, axis=0)
    #求取点云质心并进行归一化
    source_relative = source_points_nearest - source_center
    target_relative = target_points_nearest - target_center
    #进行SVD分解
    W = np.dot(source_relative.T, target_relative)
    U, _, Vt = np.linalg.svd(W)
    rotation = np.dot(Vt.T, U.T)
    #求取平移量t
    translation = target_center - np.dot(rotation, source_center)
    
    transformation = np.identity(4)
    transformation[:3, :3] = rotation
    transformation[:3, 3] = translation.T
    
    return transformation
#寻找最近邻函数
def find_correspondences(source_points, target_points):
    """Find corresponding points in target cloud for source points."""
    valid_indices = np.isfinite(source_points).all(axis=1)
    cleaned_source_points = source_points[valid_indices]
    cleaned_target_points = target_points[valid_indices]
    #构造KDTree，进行最近邻查找
    kdtree = KDTree(cleaned_target_points)
    _, indices = kdtree.query(cleaned_source_points)
    correspondences = target_points[indices]
    
    return correspondences

# def find_correspondences(source_points, target_points):
#     """Find corresponding points in target cloud for source points using FLANN."""
#     flann = pyflann.FLANN()
#     parameters = flann.build_index(target_points, algorithm='kdtree', trees=4)
    
#     _, indices = flann.nn_index(source_points, num_neighbors=1, checks=8)
#     correspondences = target_points[indices.flatten()]
    
#     return correspondences
#应用变换矩阵
def apply_transformation(points, transformation):
    """Apply transformation to points."""
    transformed_points = np.dot(transformation[:3, :3], points) + transformation[:3, 3].reshape(3, 1)
    return transformed_points
#保存变换矩阵
def save_transformation(transformation, filename):
    """Save transformation matrix to a text file."""
    np.savetxt(filename, transformation)
#保存运动轨迹
def save_trajectory(trajectory, filename):
    """Save robot trajectory to a text file."""
    with open(filename, "w") as f:
        for i, pose in enumerate(trajectory):
            f.write(f"Frame {i}:\n")
            f.write(np.array2string(pose, precision=6, suppress_small=True))
            f.write("\n\n")
#保存变换后的点云
def save_point_cloud(point_cloud, filename):
    """Save merged point cloud data to a PLY file."""
    vertex = np.array([tuple(point) for point in point_cloud], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    el = PlyElement.describe(vertex, 'vertex')
    PlyData([el]).write(filename)

def main():
    num_frames = 10
    point_clouds = [load_point_cloud(f"{i}.ply") for i in range(num_frames)]
    predict_points = [np.array([0,0,0])]
    registered_point_clouds = [point_clouds[0]]
    transformations = []
    T_all = np.eye(4)
    error = 0
    for i in range(num_frames - 1):
        source_points = point_clouds[0]
        target_points = point_clouds[i + 1]
        registered_points, transformation = iterative_closest_points(source_points, target_points)
        registered_point_clouds.append(registered_points)
        T_all = np.dot(transformation , T_all)
        print(transformation)
        predict_point = transformation[:-1, :3]
        # predict_points.append(predict_point)
        transformations.append(transformation)

    for i, transformation in enumerate(transformations):
        save_transformation(transformation, f"transformation_{i}_{i + 1}.txt")

    trajectory = [np.identity(4)]
    for transformation in transformations:
        trajectory.append(np.dot(trajectory[-1], transformation))
    #保存定位数据
    save_trajectory(trajectory, "cloud_trajectory.txt")

    merged_point_cloud = np.concatenate(registered_point_clouds)
    save_point_cloud(merged_point_cloud, "merged_point_cloud.ply")
    print('predict:',predict_points)
    Correct_points = [[0,0,0],
               [0.0459,1.1082,0],
               [0.1924,2.2068,0],
               [0.3375,3.3133,0],
               [0.3970,4.01,0],
               [0.3145,5.0264,0],
               [0.2742,5.9423,0],
               [0.1316,6.8498,9],
               [0.1663,8.1005,0],
               [0.1687,9.0731,0]
               ]
    predict_points= [[0,0,0],
               [0.044232,1.085925,0],
               [0.19022,2.181384,0],
               [ 0.335301, 3.389808,0],
               [0.409749, 4.32209,0],
               [0.343737, 5.192588,0],
               [ 0.273041,6.135863,0],
               [0.239555,7.053531,9],
               [0.196093,8.137574,0],
               [0.259797,9.170733,0]
               ]
    # print(Correct_points)
    # 将列表转换为 NumPy 数组
    Correct_points = np.array(Correct_points)
    predict_points = np.array(predict_points)
# 计算 ATE 误差
    error = np.mean(np.linalg.norm(Correct_points - predict_points, axis=1))

# 输出结果
    print("ATE 误差:", error)
if __name__ == "__main__":
    main()
