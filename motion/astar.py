from scipy.spatial import KDTree
import numpy as np
import random
import math
import time


class Node(object):
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent


class AStar(object):
    def __init__(self, N_SAMPLE=100, KNN=10, MAX_EDGE_LEN=5000):
        self.N_SAMPLE = N_SAMPLE
        self.KNN = KNN
        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 200
        self.avoid_dist = 200

    def plan(self, vision, start_x, start_y, goal_x, goal_y):
        # Obstacles
        obstacle_x = [-999999]
        obstacle_y = [-999999]
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                obstacle_x.append(robot_blue.x)
                obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                obstacle_x.append(robot_yellow.x)
                obstacle_y.append(robot_yellow.y)
        # Obstacle KD Tree
        obstree = KDTree(np.vstack((obstacle_x, obstacle_y)).T)
        # Sampling
        sample_x, sample_y = self.sampling(start_x, start_y, goal_x, goal_y, obstree)
        # Generate Roadmap
        road_map = self.generate_roadmap(sample_x, sample_y, obstree)
        # Search Path
        path_x, path_y = self.astar_search(start_x, start_y, goal_x, goal_y, sample_x, sample_y, road_map)

        return path_x, path_y, road_map, sample_x, sample_y

    def sampling(self, start_x, start_y, goal_x, goal_y, obstree):
        sample_x, sample_y = [], []

        while len(sample_x) < self.N_SAMPLE:
            tx = (random.random() * (self.maxx - self.minx)) + self.minx
            ty = (random.random() * (self.maxy - self.miny)) + self.miny

            distance, index = obstree.query(np.array([tx, ty]))

            if distance >= self.robot_size + self.avoid_dist:
                sample_x.append(tx)
                sample_y.append(ty)

        sample_x.append(start_x)
        sample_y.append(start_y)
        sample_x.append(goal_x)
        sample_y.append(goal_y)

        return sample_x, sample_y

    def generate_roadmap(self, sample_x, sample_y, obstree):
        road_map = []
        nsample = len(sample_x)
        sampletree = KDTree(np.vstack((sample_x, sample_y)).T)

        for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):
            distance, index = sampletree.query(np.array([ix, iy]), k=nsample)
            edges = []

            for ii in range(1, len(index)):
                nx = sample_x[index[ii]]
                ny = sample_y[index[ii]]

                # check collision
                if not self.check_obs(ix, iy, nx, ny, obstree):
                    edges.append(index[ii])

                if len(edges) >= self.KNN:
                    break

            road_map.append(edges)

        return road_map

    def check_obs(self, ix, iy, nx, ny, obstree):
        x = ix
        y = iy
        dx = nx - ix
        dy = ny - iy
        angle = math.atan2(dy, dx)
        dis = math.hypot(dx, dy)

        if dis > self.MAX_EDGE_LEN:
            return True

        step_size = self.robot_size + self.avoid_dist
        steps = round(dis/step_size)
        for i in range(steps):
            distance, index = obstree.query(np.array([x, y]))
            if distance <= self.robot_size + self.avoid_dist:
                return True
            x += step_size * math.cos(angle)
            y += step_size * math.sin(angle)

        # check for goal point
        distance, index = obstree.query(np.array([nx, ny]))
        if distance <= self.robot_size + self.avoid_dist:
            return True

        return False

    def astar_search(self, start_x, start_y, goal_x, goal_y, sample_x, sample_y, road_map):
        path_x, path_y = [], []
        start_node = Node(start_x, start_y, 0.0, -1)
        goal_node = Node(goal_x, goal_y, 0.0, -1)

        open_set = {len(road_map) - 2: start_node}
        close_set = {}

        while open_set:
            current_id = min(open_set, key=lambda o: open_set[o].cost + self.heuristic(open_set[o], goal_node))
            current_node = open_set[current_id]

            if current_id == len(road_map) - 1:
                goal_node.cost = current_node.cost
                goal_node.parent = current_node.parent
                break

            del open_set[current_id]
            close_set[current_id] = current_node

            for neighbor_id in road_map[current_id]:
                neighbor_node = Node(sample_x[neighbor_id], sample_y[neighbor_id], 0.0, current_id)
                neighbor_node.cost = current_node.cost + math.hypot(current_node.x - neighbor_node.x, current_node.y - neighbor_node.y)

                if neighbor_id in close_set:
                    continue

                if neighbor_id in open_set:
                    if open_set[neighbor_id].cost <= neighbor_node.cost:
                        continue

                open_set[neighbor_id] = neighbor_node

        if goal_node.parent == -1:
            return [], []

        path_node = goal_node
        while path_node.parent != -1:
            path_x.append(path_node.x)
            path_y.append(path_node.y)
            path_node = close_set[path_node.parent]

        path_x.append(start_x)
        path_y.append(start_y)
        path_x.reverse()
        path_y.reverse()

        return path_x, path_y
# math.hypot(x, y) 函数用于计算两个点之间的欧几里得距离。
    def heuristic(self, current_node, goal_node):
        return math.hypot(current_node.x - goal_node.x, current_node.y - goal_node.y)
# 切比雪夫距离（Chebyshev Distance）
# def heuristic(self, current_node, goal_node):
#     return max(abs(current_node.x - goal_node.x), abs(current_node.y - goal_node.y))
# # 欧几里得距离的平方（Squared Euclidean Distance）
# def heuristic(self, current_node, goal_node):
#     return (current_node.x - goal_node.x)**2 + (current_node.y - goal_node.y)**2
