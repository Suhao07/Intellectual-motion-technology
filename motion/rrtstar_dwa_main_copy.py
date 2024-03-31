import os, sys
import math
from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs

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

# TO DO:
# 1.KDTree搜索
# 2.速度/加速度反馈控制
# 3.对障碍物的速度估计
# 4.vision优化
class Robot:
    def __init__(self):
        self.vision = Vision()
        self.action = Action()
        self.debugger = Debugger()
        # self.planner = PRM(N_SAMPLE=1000)
        self.planner = RRT_star()

        # 添加线程锁，确保多线程操作时对共享数据的访问安全
        self.lock = threading.Lock()

    def moveto(self, goal_x, goal_y):
        # 1. path planning & velocity planning
        
        # visit target points in correct order
        step_index = 0              # 记录当前目标点在路径中的索引
        change_target_flag = False      # 记录是否转换跟踪点
        while True:
            start_x, start_y = self.vision.my_robot.x, self.vision.my_robot.y
            # boundary check
            # 边界检查 TODO:得到场地宽度
            if -999999 < start_x < 999999 and -999999 < start_y < 999999:
                path = self.planner.plan(vision=self.vision, start_x=start_x, start_y=start_y,end_x=goal_x, end_y=goal_y)
                if path is not None:
                    break


        dwa = DWA(goal_x, goal_y, vx_max=3000, vx_min = 0)

        
        while (step_index <= len(path) - 1):
            path_target = path[step_index]   
            path_x = path_target[0]
            path_y = path_target[1]
            path_x_list, path_y_list = list(zip(*path))
            dwa.set_target(path_x, path_y)
            
            total_move_time = 0
            
            while True:
                # 更新版本信息
                dwa.update_vision(self.vision)

                if dwa.if_reach_target():
                    # 更换目标点
                    change_target_flag = False
                    break
                
                # 如果移动时间过长，且与目标点距离过远，则停止追踪当前目标点并重新选择目标点
                if total_move_time > self.planner.move_max_time and \
                   change_target_flag == False and \
                   step_index < len(path) - 2 and \
                   self.planner.away_from_target(path_x, path_y, self.vision.my_robot.x, self.vision.my_robot.y):
                    next_step_index = self.planner.find_next_target(path, step_index, self.vision.my_robot.x, self.vision.my_robot.y)
                    step_index = next_step_index
                    change_target_flag = True
                    break

                # 如果更换了目标点后依然较长时间没有移动到，则继续更换目标点
                if total_move_time > self.planner.move_max_time_long and change_target_flag == True:
                    change_target_flag = False

                vx, vw, pred_x, pred_y = dwa.navigate()

                vx = self.start_adj(step_index, vx, vw)

                vx, vw = self.end_adj(step_index, path, vx, vw)

                # 2. send command
                self.action.sendCommand(vx=vx, vy=0, vw=vw)
               

                # 3. draw debug msg
                package = Debug_Msgs()
                # self.debugger.draw_points(package, path_x_list, path_y_list)
                # self.debugger.draw_circle(package, path_x, path_y, radius=100)
                self.debugger.send(package)

                time.sleep(0.01)
                total_move_time += 0.01
                
            self.action.sendCommand(vx=0, vy=0, vw=0)

            if change_target_flag == False:
                step_index += 1


        stop_time = time.time()
        time.sleep(0.5)

        return True
    
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
        robot.moveto(-2400, -1500)
        robot.moveto(2400, 1500)
