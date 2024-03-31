import os, sys
from math import cos, sin, atan2, hypot, sqrt
# sys.path.append("sys")
from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs

import time
import numpy as np
from prm import PRM
from dwa import DWA


class Robot:
    def __init__(self):
        self.vision = Vision()
        self.action = Action()
        self.debugger = Debugger()
        self.planner = PRM(N_SAMPLE=1000)
    

    def goto(self, goal_x, goal_y):
        # 1. path planning & velocity planning
        start_time = time.time()
        
        while True:
            start_x, start_y = self.vision.my_robot.x, self.vision.my_robot.y
            # print("start: ", start_x, start_y)
            path_x, path_y, road_map, sample_x, sample_y = self.planner.plan(vision=self.vision, start_x=start_x, start_y=start_y, 
                                                                             goal_x=goal_x, goal_y=goal_y)
            if len(path_x) > 0 and len(path_y) > 0:
                break

        etime1 = time.time()
        print("path planning time: ", etime1 - start_time)

        dwa = DWA(goal_x, goal_y, vx_max=3000, vx_min = -3000)

        # visit target points in correct order
        for target_x, target_y in zip(reversed(path_x), reversed(path_y)):     
            dwa.set_target(target_x, target_y)
            
            while True:
                # update version information
                dwa.update_vision(self.vision)
        
                if dwa.if_reach_target():
                    # change target point
                    break
                
                vx, vw, pred_x, pred_y = dwa.navigate()
        
                # 2. send command
                self.action.sendCommand(vx=vx, vy=0, vw=vw)

                # 3. draw debug msg
                package = Debug_Msgs()
                # self.debugger.draw_all(sample_x, sample_y, road_map, path_x, path_y)
                self.debugger.draw_points(package, path_x, path_y)
                self.debugger.draw_circle(package, target_x, target_y, radius=100)
                self.debugger.send(package)

                time.sleep(0.01)
                
            self.action.sendCommand(vx=0, vy=0, vw=0)
            
        stop_time = time.time()
        
        return stop_time - start_time


if __name__ == '__main__':
    robot = Robot()
    
    while True:    
        robot.goto(-2400, -1500)
        robot.goto(2400, 1500)

        
