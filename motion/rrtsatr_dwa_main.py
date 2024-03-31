import os, sys
import math
from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
from scipy.interpolate import CubicSpline

import time
import numpy as np
from rrtstar import RRT_star
# from RRTStar_copy import RRT_star
from dwa import DWA

import threading
import concurrent.futures

# "目前用到的优化方法："
# "1.多线程并行处理"
# 2.nump向量优化搜索
#3.进行path的平滑化（样条插值），效果不好
# 4.速度和加速度反馈控制
# 5.对obstacles的速度复杂预测器（短时间内的匀加速模型）
# TODO：
# 1.vision的debugger可视化功能
# 2.增加局部最优处理
# 3.使用KDtree或Quadtree储存障碍物进行遍历，提高效率
class Robot:
    def __init__(self):
        self.vision = Vision()
        self.action = Action()
        self.debugger = Debugger()
        # self.planner = PRM(N_SAMPLE=1000)
        self.planner = RRT_star()
        self.boundary_x=99999
        self.boundary_y=99999
        # 添加线程锁，确保多线程操作时对共享数据的访问安全
        self.lock = threading.Lock()

    def moveto(self, goal_x, goal_y):
        # 1. path planning & velocity planning
        start_time = time.time()  # 记录开始时间
        while True:
            start_x, start_y = self.vision.my_robot.x, self.vision.my_robot.y
# 边界检查
            if abs(start_x)<self.boundary_x and abs(start_y)<self.boundary_y:
                path = self.planner.plan(vision=self.vision, start_x=start_x, start_y=start_y,end_x=goal_x, end_y=goal_y)
                if path is not None:
                    break

        dwa = DWA(goal_x, goal_y, vx_max=1200, vx_min = 0)

        # visit target points in correct order
        current_index = 0              # 记录当前目标点在路径中的索引
        change_target_flag = False      # 记录是否跳过当前目标点
        #遍历path并设置目标点
        while (current_index < len(path) ):
            path_target = path[current_index]   
            path_x = path_target[0]
            path_y = path_target[1]
            path_x_list, path_y_list = list(zip(*path))
            dwa.set_target(path_x, path_y)#设置目标点
            
            total_time = 0
            
            while True:
                step_start_time = time.time()  # 记录开始时间
                # 更新版本信息
                dwa.update_vision(self.vision)

                if dwa.reach_target():
                    # 更换目标点
                    change_target_flag = False
                    break
                
                # 如果移动时间过长，且与目标点距离过远，则停止追踪当前目标点并重新选择目标点
                if total_time > self.planner.move_max_time and \
                   not change_target_flag  and \
                   current_index < len(path) - 2 and \
                   self.planner.away_from_target(path_x, path_y, self.vision.my_robot.x, self.vision.my_robot.y):
                    next_current_index = self.planner.find_next_target(path, current_index, self.vision.my_robot.x, self.vision.my_robot.y)
                    current_index = next_current_index
                    change_target_flag = True
                    break

                # 如果更换了目标点后依然较长时间没有移动到，则继续更换目标点
                if total_time > self.planner.move_max_time_long and change_target_flag == True:
                    change_target_flag = False
                #TODO：局部最优问题解决
                #try:如果在一个地方停滞很久，则重新寻找目标
                #predict存取了但未在此使用
                vx, vw, pred_x, pred_y = dwa.navigate()
                

                # 3. draw debug msg
                package = Debug_Msgs()
                self.debugger.draw_points(package, path_x_list, path_y_list)
                self.debugger.draw_circle(package, path_x, path_y, radius=90)
                # if current_index>0 and current_index <len(path)-1:
                    # self.debugger.draw_lines(package,path_x_list[current_index-1],path_y_list[current_index-1],path_x_list[current_index],path_y_list[current_index])
                # print(path_x_list[current_index])
                self.debugger.send(package)

                step_end_time = time.time()  # 记录开始时间
                step_time = step_end_time - step_start_time
                total_time += step_time
                # 2. send command
                self.action.sendCommand(vx=vx, vy=0, vw=vw,step_time=step_time)
               
            self.action.sendCommand(vx=0, vy=0, vw=0)

            if change_target_flag == False:
                current_index += 1

        stop_time = time.time()  # 记录结束时间
        run_time = stop_time - start_time  # 计算运行时间
        print("运行时间:", run_time)
        # 执行之后的sleep
        time.sleep(0.4)

        return True

if __name__ == '__main__':
    robot = Robot()
    
    while True:    
        robot.moveto(-2500, -1600)
        robot.moveto(2500, 1600)
