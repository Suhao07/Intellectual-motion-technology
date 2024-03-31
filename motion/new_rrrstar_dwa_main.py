import os
import sys
import math
import time
import threading
import concurrent.futures
from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
from evaluator import Evaluator
from rrtstar import RRT_star
from dwa import DWA

class Robot:
    def __init__(self):
        self.vision = Vision()
        self.action = Action()
        self.debugger = Debugger()
        self.planner = RRT_star()
        self.lock = threading.Lock()

    def goto(self, goal_x, goal_y):
        start_time = time.time()
        
        # 多线程并行计算备选路径
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # 定义一个字典，用于存储每个备选路径及其对应的成本
            path_costs = {}
            # 定义一个列表，用于存储每个线程的Future对象
            futures = []
            
            for i in range(5):  # 启动5个线程并行计算备选路径
                future = executor.submit(self.planner.plan, 
                                         vision=self.vision, 
                                         start_x=self.vision.my_robot.x, 
                                         start_y=self.vision.my_robot.y, 
                                         end_x=goal_x, 
                                         end_y=goal_y, 
                                         method='RRT*')
                futures.append(future)
            
            # 等待所有线程完成计算
            for future in concurrent.futures.as_completed(futures):
                path = future.result()
                if path is not None:
                    # 计算路径成本，并存储到字典中
                    cost = self.calculate_path_cost(path)
                    path_costs[path] = cost
        
        # 从备选路径中选择成本最低的路径作为最优路径
        best_path = min(path_costs, key=path_costs.get)
        
        # 执行移动操作
        self.execute_path(best_path)
        
        end_time = time.time()
        return end_time - start_time

    def calculate_path_cost(self, path):
        # 计算路径成本的简化实现，可以根据具体需求进行修改
        return len(path)

    def execute_path(self, path):
        # 执行路径移动操作的简化实现，可以根据具体需求进行修改
        pass

    def start_adj(self, step_index, vx, vw):
        if step_index <= 2:
            vx = 0.5 * vx
        return vx

    def end_adj(self, step_index, path, vx, vw):
        if step_index >= len(path) - 3:
            vx = 0.9 * vx
            vw = 0.9 * vw
        return vx, vw

if __name__ == '__main__':
    robot = Robot()
    
    while True:    
        robot.goto(-2400, -1500)
        robot.goto(2400, 1500)
