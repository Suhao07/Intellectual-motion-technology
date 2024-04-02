import random
import math
import threading
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import KDTree
#节点
class Node(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

#RRT*算法
class RRT_star(object):
    def __init__(self):
        self.x_min = -3000
        self.x_max = 3000
        self.y_min = -3000
        self.y_max = 3000

        self.sample_rate = 0.68         # 随机采样概率
        self.robot_radius = 90         # 机器人半径
        self.step_size = 360            # 走一步的步长
        self.near_radius = 2000         # 查找近邻点X_near的半径
        self.max_iter = 750             # 最大迭代次数
        self.arrive_threshold = 50      # 到达目标点的判断阈值
        self.min_jump_step = 3          # 跳步的最大步长
        self.max_jump_step = 8          # 跳步的最大步长
        self.move_max_time = 0.02       # DWA单步移动的最大时间
        self.move_max_time_long = 0.09  # DWA跳步后单步移动的最大时间

        self.vision = None
        self.start = Node(0, 0)       # 开始节点
        self.end = Node(0, 0)         # 目标节点
        self.obstacles = []           # 障碍物列表
        self.obstacles_array = np.array([])  # 存储障碍物的numpy数组
        self.collide_results = []  # 存储碰撞检测结果
        # self.quadtree = Quadtree(self.x_min, self.x_max, self.y_min, self.y_max)
        # self.executor = ThreadPoolExecutor(max_workers=4)  # 使用线程池
# RRT* 路径规划
    def plan(self, vision, start_x, start_y, end_x, end_y):
        # 根据vision设置障碍物
        self.vision = vision
        self.set_obstacle(self.vision)
        node_list = [self.start]
        node_end_list = []
        self.start.x = start_x
        self.start.y = start_y
        self.end.x = end_x
        self.end.y = end_y
        

        for i in range(self.max_iter):
            # 并行化随机采样
            node_rand = self.parallel_sample()
            # 找到树中离采样点最近的树节点
            node_nearest = self.nearest(node_list, node_rand)
            # 根据机器人执行能力生成新节点和新的路径
            node_new = self.steer(node_rand, node_nearest)
            # 如果新节点与其父节点连线无碰, 则加入树中
            if self.check_collision(node_new, node_new.parent):
                
                    # 选择新节点的所有邻节点
                    Nodes_near = self.neighbor_nodes(node_list, node_new)
                    # 对邻节点中的每个节点, 计算以其作为新节点x_new的父节点的cost
                    # 找到cost最小的节点，以其作为新节点x_new的父节点
                    node_min, cost_min = self.choose_parent(node_new, Nodes_near)
                    self.rewire(node_new, node_min, cost_min)

                    node_list.append(node_new)
                    # 如果新节点到达目标点, 则结束迭代, 返回路径
                    if self.arrived(node_new.x, node_new.y):
                        node_end_list.append(node_new)

        # 找到最优路径
        if(len(node_end_list) > 0):
            final_node_end = self.find_best_path(node_end_list)
            path = self.find_path(final_node_end)
            # smooth_path = self.smooth_path(path)
            return path
        else:
            return None
# 并行化随机采样
    def parallel_sample(self):
        if random.random() <= self.sample_rate:
            # 生成随机数
            x_rand = random.uniform(self.x_min, self.x_max)
            y_rand = random.uniform(self.y_min, self.y_max)
        else:
            x_rand = self.end.x
            y_rand = self.end.y
        return Node(x_rand, y_rand)
#  并行化碰撞检测
    def parallel_collision_detector(self, node_1, node_2, obstacles):
    # 计算斜率和截距
        k = (node_2.y - node_1.y) / (node_2.x - node_1.x)
        b = node_1.y - k * node_1.x
        # 计算距离
        dist = abs(self.obstacles_array[:, 1] - k * self.obstacles_array[:, 0] - b) / np.sqrt(1 + k ** 2)
        # 计算垂足
        x0 = (self.obstacles_array[:, 0] + k * self.obstacles_array[:, 1] - k * b) / (1 + k ** 2)
        y0 = k * x0 + b
        # 判断是否碰撞
        in_two_flag = ((np.minimum(node_1.x, node_2.x) - 250) <= x0) & \
                      (x0 <= (np.maximum(node_1.x, node_2.x) + 250)) & \
                      ((np.minimum(node_1.y, node_2.y) - 250) <= y0) & \
                      (y0 <= (np.maximum(node_1.y, node_2.y) + 250))
        collide_flag = np.any(in_two_flag & (dist <= self.robot_radius))
        self.collide_results.append(not collide_flag)  # 将碰撞检测结果存储在共享的数据结构中
        return not collide_flag
#  多线程并行碰撞检测
    def parallel_collision_detector_threaded(self, node_1, node_2, obstacles):
        threads = []
        results = []
        def collision_detector(obstacle, node_1, node_2, result):
    # 将障碍物表示为向量
            obstacle_vector = np.array([obstacle[0], obstacle[1]])

    # 计算节点到障碍物的向量
            node_vector = np.array([node_1.x, node_1.y])
            obstacle_start_vector = np.array([node_2.x, node_2.y])
            node_to_obstacle_vector = obstacle_start_vector - node_vector

    # 计算投影
            projection_length = np.dot(node_to_obstacle_vector, obstacle_vector) / np.linalg.norm(obstacle_vector)

    # 判断碰撞
            if 0 <= projection_length <= np.linalg.norm(obstacle_vector) and np.linalg.norm(node_to_obstacle_vector) <= self.robot_radius:
                result.append(False)
            else:
                result.append(True)
        # 创建线程
        for obstacle in obstacles:
            result = []
            thread = threading.Thread(target=collision_detector, args=(obstacle, node_1, node_2, result))
            threads.append(thread)
            results.append(result)

        # 启动线程
        for thread in threads:
            thread.start()

        # 等待线程执行完成
        for thread in threads:
            thread.join()

        # 检查碰撞结果
        for result in results:
            if not result[0]:
                return False

        return True
# 从x_nearest出发, 向x_rand方向前进step_size, 生成x_new
    def steer(self, node_rand, node_nearest):
        dist = self.dist_node(node_nearest, node_rand)
        if dist <= self.step_size:
            return node_rand
        else:
            node_new_x = node_nearest.x + (self.step_size / dist) * (node_rand.x - node_nearest.x)
            node_new_y = node_nearest.y + (self.step_size / dist) * (node_rand.y - node_nearest.y)
            node_new = Node(node_new_x, node_new_y)
            node_new.parent = node_nearest
            return node_new
# 并行化路径搜索
    def parallel_path_search(self, vision, start_x, start_y, end_x, end_y):
        # 根据vision设置障碍物
        self.vision = vision
        self.set_obstacle(self.vision)

        self.start.x = start_x
        self.start.y = start_y
        self.end.x = end_x
        self.end.y = end_y
        node_list = [self.start]
        node_end_list = []

        for i in range(self.max_iter):
            # 并行化随机采样
            node_rand = self.parallel_sample()
            # 找到树中离采样点最近的树节点
            node_nearest = self.nearest(node_list, node_rand)
            # 根据机器人执行能力生成新节点和新的路径
            node_new = self.steer(node_rand, node_nearest)
            # 并行化碰撞检测
            if self.parallel_collision_detector_threaded(node_new, node_new.parent, self.obstacles):
                ### RRT*: 
                # elif method == 'RRT*':
                    # 选择新节点的所有邻节点
                    Nodes_near = self.near(node_list, node_new)
                    # 对邻节点中的每个节点, 计算以其作为新节点x_new的父节点的cost
                    # 找到cost最小的节点，以其作为新节点x_new的父节点
                    node_min, cost_min = self.choose_parent(node_new, Nodes_near)
                    self.rewire(node_new, node_min, cost_min)

                    node_list.append(node_new)
                    # 如果新节点到达目标点, 则结束迭代, 返回路径
                    if self.arrived(node_new.x, node_new.y):
                        node_end_list.append(node_new)

        # 找到最优路径
        if(len(node_end_list) > 0):
            final_node_end = self.find_best_path(node_end_list)
            path = self.find_path(final_node_end)
            return path
        else:
            return None
# 判断是否到达目标点
    def arrived(self, node_x, node_y):
        node = Node(node_x, node_y)
        if self.dist_node(node, self.end) <= self.arrive_threshold:
            return True
        else:
            return False
#判断连线是否存在障碍物
    def check_collision(self, node_1, node_2):
        if not node_2:
            return False

    # 计算节点之间的向量差
        delta_x = node_2.x - node_1.x
        delta_y = node_2.y - node_1.y

    # 计算节点之间的距离
        dist = np.sqrt(delta_x ** 2 + delta_y ** 2)

    # 计算斜率和截距
        k = delta_y / delta_x
        b = node_1.y - k * node_1.x

    # 计算与障碍物的距离
        obstacle_distances = np.abs(self.obstacles_array[:, 1] - k * self.obstacles_array[:, 0] - b) / np.sqrt(1 + k ** 2)

    # 检测是否有任何距离小于阈值的距离
        too_close_flag = np.any(obstacle_distances <= self.robot_radius)

        return not too_close_flag

    # 根据vision设置障碍物
    def set_obstacle(self, vision):
        # 清空障碍物列表
        obstacles = []
        # 设置蓝色、黄色机器人障碍物
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                obstacles.append([robot_yellow.x, robot_yellow.y])
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                obstacles.append([robot_blue.x, robot_blue.y])
        self.obstacles_array = np.array(obstacles)
    #   计算两个点间的欧氏距离  
    def dist_node(self, node_1, node_2):
        dist = math.sqrt((node_1.x - node_2.x)**2 + (node_1.y - node_2.y)**2)
        return dist
#  找到树中离采样点x_rand近的树节点
    def nearest(self, node_list, node_rand):
        node_x = [node.x for node in node_list]
        node_y = [node.y for node in node_list]
        node_tree = KDTree(np.vstack((node_x, node_y)).T)
        _, index = node_tree.query(np.array([node_rand.x, node_rand.y]), k=1)
        return node_list[index]

#  选择x_new的邻域内所有节点
    def neighbor_nodes(self, node_list, node_new):
        node_x = [node.x for node in node_list]
        node_y = [node.y for node in node_list]
        node_tree = KDTree(np.vstack((node_x, node_y)).T)
        indices = node_tree.query_ball_point(np.array([node_new.x, node_new.y]), self.near_radius)
        Nodes_near = [node_list[i] for i in indices]
        return Nodes_near
# 对树进行重连, 将cost最小的节点作为新节点的父节点
    def rewire(self, node_new, node_new_parent, cost_new):
        node_new.parent = node_new_parent
        node_new.cost = cost_new
# 对邻节点中的每个节点, 计算以其作为新节点x_new的父节点的cost, 
#         找到cost最小的节点, 以其作为新节点x_new的父节点
    def choose_parent(self, node_new, Nodes_near):
        node_min = node_new.parent
        cost_min = node_new.cost

        for node_near in Nodes_near:
            cost = node_near.cost + self.dist_node(node_near, node_new)
            if cost < cost_min and self.parallel_collision_detector_threaded(node_near, node_new, self.obstacles):
                node_min = node_near
                cost_min = cost
        
        return node_min, cost_min
# 从最后一个节点开始回溯路径
    def find_path(self, end_node):
        path = [end_node]
        while path[-1].parent:
            path.append(path[-1].parent)
        
        path = reversed(path)
        path = [(node.x, node.y) for node in path]
        
        return path
# 找到终点节点列表中cost最小的节点
    def find_best_path(self, node_end_list):
        node_end = node_end_list[0]
        for node in node_end_list:
            if node.cost < node_end.cost:
                node_end = node
        return node_end
#  判断是否离目标点太远
    def away_from_target(self, path_x, path_y, target_x, target_y):
        dist = math.sqrt((path_x - target_x)**2 + (path_y - target_y)**2)
        if dist > (self.robot_radius):
            return True
        else:
            return False

#   找到下一个目标点
    def find_next_target(self, path, step_index, robot_x, robot_y):
        dist_min = float('inf')
        next_step_index = step_index + self.min_jump_step
        max_step_index = min(step_index + self.max_jump_step, len(path) - 1)

        for i in range(step_index + 1, max_step_index + 1):
            node_1 = Node(robot_x, robot_y)
            node_2 = Node(path[i][0], path[i][1])
            dist = self.dist_node(node_1, node_2)
            if dist < dist_min:
                dist_min = dist
                next_step_index = i

        return next_step_index
    #进行样条插值，将路径平滑化
    def smooth_path(self,path):
    # 将路径拆分为x和y坐标
        x_coords, y_coords = zip(*path)

    # 使用样条插值对x和y坐标进行平滑化
        spline_x = CubicSpline(range(len(x_coords)), x_coords)
        spline_y = CubicSpline(range(len(y_coords)), y_coords)

    # 在原始路径的范围内生成新的平滑化路径
        new_path = [(spline_x(i), spline_y(i)) for i in np.linspace(0, len(path) - 1, num=100)]

        return new_path