from vision import Vision
from action import Action
from debug import Debugger
from matplotlib import pyplot as plt
from zss_debug_pb2 import Debug_Msgs

import time
import numpy as np

import threading
import concurrent.futures

class Evaluator:
    def __init__(self):
        self.vision = None

        self.x_rec = np.array([])   # x
        self.theta_rec = np.array([])   # theta
        self.v_rec = np.array([])   # 线速度
        self.w_rec = np.array([])   # 角速度
        self.a_rec = np.array([])   # 线速度加速度
        self.b_rec = np.array([])   # 角速度加速度
        self.t_rec = np.array([])
        
        self.start_time = None
        self.last_update_time = None
        self.current_time = 0
        
        self.time_interval = None
        
        self.lock = threading.Lock()  # 添加线程锁，防止多线程同时访问共享数据
        
    def recording(self, vision: Vision=None):
        
        self.vision = vision
        
        if self.start_time == None:
            # 初始化
            self.start_time = time.time() 
            self.last_update_time = None
            return
        else:
            self.last_update_time = self.current_time
            self.current_time = time.time() - self.start_time
        
        self.time_interval = self.current_time - self.last_update_time
        
        with self.lock:  # 使用线程锁确保线程安全访问共享数据
            if len(self.v_rec) > 2 and len(self.w_rec) > 2:
                self.a_rec = np.append(self.a_rec, (self.v_rec[-1] - self.v_rec[-2]) / self.time_interval)
                self.b_rec = np.append(self.b_rec, (self.v_rec[-1] - self.w_rec[-2]) / self.time_interval)
            else:
                self.a_rec = np.append(self.a_rec, 0)
                self.b_rec = np.append(self.b_rec, 0)
                
            if len(self.x_rec) > 2 and len(self.theta_rec) > 2:
                self.v_rec = np.append(self.v_rec, (vision.my_robot.x - self.x_rec[-1]) / self.time_interval)
                self.w_rec = np.append(self.w_rec, (vision.my_robot.orientation - self.theta_rec[-1]) / self.time_interval)
            else:
                self.v_rec = np.append(self.v_rec, 0)
                self.w_rec = np.append(self.w_rec, 0)
                                       
            self.t_rec = np.append(self.t_rec, self.current_time)
            self.x_rec = np.append(self.x_rec, vision.my_robot.x)
            self.theta_rec = np.append(self.theta_rec, vision.my_robot.orientation)
        
    def plot_history(self):
        plt.plot(self.t_rec, self.v_rec, label='v')
        plt.show()

    def parallel_collision_detection_threaded(self, node1, node2, obstacles):
        """
        并行化碰撞检测过程，多线程检测节点间的碰撞
        """
        # 碰撞检测函数，用于单个线程执行
        def collision_detection(node1, node2, obstacles):
            for obstacle in obstacles:
                # 计算斜率和截距
                k = (node2.y - node1.y) / (node2.x - node1.x)
                b = node1.y - k * node1.x
                
                # 计算距离
                dist = abs(obstacle[1] - k * obstacle[0] - b) / math.sqrt(1 + k ** 2)
                # 计算垂足
                x0 = (obstacle[0] + k * obstacle[1] - k * b) / (1 + k ** 2)
                y0 = k * x0 + b
                in_two_flag = ((min(node1.x, node2.x) - 250) <= x0 and x0 <= (max(node1.x, node2.x) + 250) and \
                            (min(node1.y, node2.y) - 250) <= y0 and y0 <= (max(node1.y, node2.y) + 250))
                # 如果垂足在两点间且距离小于阈值, 则碰撞
                if in_two_flag and (dist <= self.robot_radius):
                    return False
            return True
        
        # 同时处理碰撞检测任务
        with concurrent.futures.ThreadPoolExecutor() as executor:
            results = [executor.submit(collision_detection, node1, node2, obstacles) for _ in range(len(obstacles))]
        
        # 检查所有碰撞检测结果
        for result in concurrent.futures.as_completed(results):
            if not result.result():
                return False
        return True
