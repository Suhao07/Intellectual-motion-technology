import random
import math
import numpy as np

class Node(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

class KDNode(object):
    def __init__(self, point, axis, left=None, right=None):
        self.point = point
        self.axis = axis
        self.left = left
        self.right = right

class KDTree(object):
    def __init__(self, points):
        self.root = self.build_kdtree(points)

    def build_kdtree(self, points, depth=0):
        if not points:
            return None
        k = len(points[0])
        axis = depth % k
        points.sort(key=lambda x: x[axis])
        median = len(points) // 2
        return KDNode(
            point=points[median],
            axis=axis,
            left=self.build_kdtree(points[:median], depth + 1),
            right=self.build_kdtree(points[median + 1:], depth + 1)
        )

    def nearest_neighbor_search(self, target, root=None, depth=0, best=None):
        if root is None:
            root = self.root
        if root is None:
            return None

        k = len(target)
        axis = depth % k

        next_branch = None
        opposite_branch = None

        if target[axis] < root.point[axis]:
            next_branch = root.left
            opposite_branch = root.right
        else:
            next_branch = root.right
            opposite_branch = root.left

        if next_branch is None:
            return root.point

        best = self.nearest_neighbor_search(target, next_branch, depth + 1, best)

        if self.distance(target, best) > abs(target[axis] - root.point[axis]):
            best = self.nearest_neighbor_search(target, opposite_branch, depth + 1, best)

        return best

    def distance(self, point1, point2):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(point1, point2)))

class RRT_star(object):
    def __init__(self):
        self.x_min = -3000
        self.x_max = 3000
        self.y_min = -3000
        self.y_max = 3000

        self.sample_rate = 0.68
        self.robot_radius = 200
        self.avoid_dist = 600
        self.step_size = 360
        self.near_radius = 2000
        self.max_iter = 750
        self.arrive_threshold = 50
        self.min_jump_step = 3
        self.max_jump_step = 8
        self.move_max_time = 0.02
        self.move_max_time_long = 0.09

        self.vision = None
        self.start = Node(0, 0)
        self.end = Node(0, 0)
        self.obstacles = []
        self.obstacles_array = np.array([])
        self.collide_results = []

        self.kd_tree = None

    def plan(self, vision, start_x, start_y, end_x, end_y):
        self.vision = vision
        self.set_obstacle(self.vision)

        self.start.x = start_x
        self.start.y = start_y
        self.end.x = end_x
        self.end.y = end_y

        points = [[obstacle[0], obstacle[1]] for obstacle in self.obstacles]
        self.kd_tree = KDTree(points)

        node_list = {self.start: 0}
        node_end_list = []

        for i in range(self.max_iter):
            node_rand = self.parallel_sample()
            node_nearest = self.nearest(node_list, node_rand)
            node_new = self.steer(node_rand, node_nearest)
            if self.check_collision(node_new, node_new.parent):
                Nodes_near = self.near(node_list, node_new)
                node_min, cost_min = self.choose_parent(node_new, Nodes_near)
                self.rewire(node_new, node_min, cost_min)
                node_list[node_new] = node_new.cost
                if self.arrived(node_new.x, node_new.y):
                    node_end_list.append(node_new)

        if len(node_end_list) > 0:
            final_node_end = self.find_best_path(node_end_list)
            path = self.find_path(final_node_end)
            return path
        else:
            return None

    def parallel_sample(self):
        if random.random() <= self.sample_rate:
            x_rand = random.uniform(self.x_min, self.x_max)
            y_rand = random.uniform(self.y_min, self.y_max)
        else:
            x_rand = self.end.x
            y_rand = self.end.y
        return Node(x_rand, y_rand)

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

    def nearest(self, node_dict, node_rand):
        nearest_point = self.kd_tree.nearest_neighbor_search([node_rand.x, node_rand.y])
        return Node(nearest_point[0], nearest_point[1])

    def near(self, node_dict, node_new):
        Nodes_near = [node for node in node_dict if self.dist_node(node, node_new) <= self.near_radius]
        return Nodes_near

    def rewire(self, node_new, node_new_parent, cost_new):
        node_new.parent = node_new_parent
        node_new.cost = cost_new

    def choose_parent(self, node_new, Nodes_near):
        node_min = node_new.parent
        cost_min = node_new.cost

        for node_near in Nodes_near:
            cost = node_near.cost + self.dist_node(node_near, node_new)
            if cost < cost_min:
                node_min = node_near
                cost_min = cost

        return node_min, cost_min

    def arrived(self, node_x, node_y):
        node = Node(node_x, node_y)
        if self.dist_node(node, self.end) <= self.arrive_threshold:
            return True
        else:
            return False

    def check_collision(self, node_1, node_2):
        if not node_2:
            return False

        delta_x = node_2.x - node_1.x
        delta_y = node_2.y - node_1.y
        dist = np.sqrt(delta_x ** 2 + delta_y ** 2)
        k = delta_y / delta_x
        b = node_1.y - k * node_1.x
        obstacle_distances = np.abs(self.obstacles_array[:, 1] - k * self.obstacles_array[:, 0] - b) / np.sqrt(1 + k ** 2)
        too_close_flag = np.any(obstacle_distances <= self.robot_radius)

        return not too_close_flag

    def set_obstacle(self, vision):
        obstacles = []
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                obstacles.append([robot_yellow.x, robot_yellow.y])
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                obstacles.append([robot_blue.x, robot_blue.y])
        self.obstacles_array = np.array(obstacles)

    def dist_node(self, node_1, node_2):
        dist = math.sqrt((node_1.x - node_2.x) ** 2 + (node_1.y - node_2.y) ** 2)
        return dist

    def find_path(self, end_node):
        path = [end_node]
        while path[-1].parent:
            path.append(path[-1].parent)

        path = reversed(path)
        path = [(node.x, node.y) for node in path]

        return path

    def find_best_path(self, node_end_list):
        node_end = node_end_list[0]
        for node in node_end_list:
            if node.cost < node_end.cost:
                node_end = node
        return node_end
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