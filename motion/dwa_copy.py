import numpy as np
import copy
import time
from concurrent.futures import ThreadPoolExecutor
import concurrent.futures

class Pos:
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = theta

class Vel:
    def __init__(self, vx: float, vw: float):
        self.vx = vx
        self.vw = vw
class Range:
    def __init__(self, min_val: float, max_val: float):
        self.min = min_val
        self.max = max_val
class DWA:
    def __init__(self, final_x, final_y, 
                 vx_min: float=-3000.0, vx_max: float=3000.0, 
                 dx_min: float=-3000.0, dx_max: float=3000.0, 
                 vw_min: float=-15, vw_max: float=10, 
                 dw_min: float=-20, dw_max: float=20, 
                 dt=0.01, simt=0.2, res=10):
        self.obstacles_history = []  
        self.obstacles = []  
        self.obstacles_predict = []  
        self.pos = Pos(0.0, 0.0, 0.0)
        self.vel = Vel(0.0, 0.0)
        self.vx_range = Range(vx_min, vx_max)
        self.dx_range = Range(dx_min, dx_max)
        self.vw_range = Range(vw_min, vw_max)
        self.dw_range = Range(dw_min, dw_max)
        self.target = None
        self.final_x = final_x
        self.final_y = final_y
        self.reach_dist = 150
        self.dangerous_dist = 300
        self.dt = dt
        self.simt = simt
        self.res = res
        self.last_update_time = 0   
        self.cur_update_time = 0

    # def update_vision(self, vision):    
    #     self.last_update_time = self.cur_update_time
    #     self.cur_update_time = time.time()
    #     time_interval = 0.033
    #     self.pos = Pos(vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation)
    #     self.obstacles_history = copy.copy(self.obstacles)
    #     self.obstacles = [Pos(robot.x, robot.y, robot.orientation) for robot in vision.blue_robot if robot.id != 0] + [Pos(robot.x, robot.y, robot.orientation) for robot in vision.yellow_robot]
        
    #     self.obstacles_predict = [Pos(obs.x, obs.y + np.sign((obs.y - obs_histroy.y) / time_interval) * 150, obs.theta) for obs, obs_histroy in zip(self.obstacles, self.obstacles_history)]
    #     return self.obstacles_predict

    def update_vision(self, vision):    
        self.last_update_time = self.cur_update_time
        self.cur_update_time = time.time()
        time_interval = self.cur_update_time - self.last_update_time
        
        # 保存上一时刻的障碍物信息
        self.obstacles_history = copy.deepcopy(self.obstacles)

        # 更新当前时刻的障碍物信息
        self.pos = Pos(vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation)
        self.obstacles = [Pos(robot.x, robot.y, robot.orientation) for robot in vision.blue_robot if robot.id != 0] + [Pos(robot.x, robot.y, robot.orientation) for robot in vision.yellow_robot]
        
        return self.obstacles_predict

    def predict_obstacle_pos(self, obstacle):
        # 获取上一时刻的障碍物位置
        prev_obstacle = self.get_previous_obstacle(obstacle)

        # 当前时刻的障碍物位置
        current_obstacle = obstacle

        # 计算时间间隔
        time_interval = self.cur_update_time - self.last_update_time

        # 计算速度变化
        vx_change = (current_obstacle.x - prev_obstacle.x) / time_interval
        vy_change = (current_obstacle.y - prev_obstacle.y) / time_interval

        # 计算加速度变化
        ax_change = (current_obstacle.vx - prev_obstacle.vx) / time_interval
        ay_change = (current_obstacle.vy - prev_obstacle.vy) / time_interval

        # 预测下一个时刻的位置（假设加速度不变）
        next_x = current_obstacle.x + current_obstacle.vx * time_interval + 0.5 * ax_change * time_interval**2
        next_y = current_obstacle.y + current_obstacle.vy * time_interval + 0.5 * ay_change * time_interval**2

        # 创建预测的障碍物对象
        predicted_obstacle = Pos(next_x, next_y, current_obstacle.theta)

        return predicted_obstacle

    def get_previous_obstacle(self, obstacle):
        # 从保存的上一时刻的障碍物信息中获取指定障碍物的上一时刻位置和速度
        for prev_obstacle in self.obstacles_history:
            if prev_obstacle.id == obstacle.id:
                return prev_obstacle
        return None


    def reach_target(self):  
        distance_to_target = np.sqrt(pow(self.target.x - self.pos.x, 2) + pow(self.target.y - self.pos.y, 2))
        return (distance_to_target < self.reach_dist) if (self.target.x == self.final_x and self.target.y == self.final_y) else (distance_to_target < self.reach_dist)
    # 欧拉法
    # def predict_pos(self, vx_sim, vw_sim):
    #     pred_pos = copy.copy(self.pos)
    #     for _ in range(int(self.simt / self.dt)):
    #         pred_pos.x += vx_sim * self.dt * np.cos(pred_pos.theta)
    #         pred_pos.y += vx_sim * self.dt * np.sin(pred_pos.theta)
    #         pred_pos.theta += vw_sim * self.dt
    #     return pred_pos
    # 四阶 Runge-Kutta 方法
    # def predict_pos(self, vx_sim, vw_sim):
    #     pred_pos = copy.copy(self.pos)

    #     dt = self.dt  # 时间步长
    #     t = 0  # 初始化时间
    #     while t < self.simt:
    #     # Runge-Kutta 方法
    #         k1 = vx_sim * dt * np.cos(pred_pos.theta)
    #         l1 = vx_sim * dt * np.sin(pred_pos.theta)
    #         m1 = vw_sim * dt
    #         k2 = (vx_sim + 0.5 * k1) * dt * np.cos(pred_pos.theta + 0.5 * m1)
    #         l2 = (vx_sim + 0.5 * k1) * dt * np.sin(pred_pos.theta + 0.5 * m1)
    #         m2 = (vw_sim + 0.5 * m1) * dt
    #         k3 = (vx_sim + 0.5 * k2) * dt * np.cos(pred_pos.theta + 0.5 * m2)
    #         l3 = (vx_sim + 0.5 * k2) * dt * np.sin(pred_pos.theta + 0.5 * m2)
    #         m3 = (vw_sim + 0.5 * m2) * dt
    #         k4 = (vx_sim + k3) * dt * np.cos(pred_pos.theta + m3)
    #         l4 = (vx_sim + k3) * dt * np.sin(pred_pos.theta + m3)
    #         m4 = (vw_sim + m3) * dt

    #         pred_pos.x += (k1 + 2 * k2 + 2 * k3 + k4) / 6
    #         pred_pos.y += (l1 + 2 * l2 + 2 * l3 + l4) / 6
    #         pred_pos.theta += (m1 + 2 * m2 + 2 * m3 + m4) / 6

    #         t += dt  # 更新时间

    #     return pred_pos
    # 改进欧拉法
    def predict_pos(self, vx_sim, vw_sim):
        pred_pos = copy.copy(self.pos)
    
        dt = self.dt  # 时间步长
        t = 0  # 初始化时间
        while t < self.simt:
        # 改进的欧拉方法（Heun's method）
            k1 = vx_sim * dt * np.cos(pred_pos.theta)
            l1 = vx_sim * dt * np.sin(pred_pos.theta)
            m1 = vw_sim * dt
        
            k2 = (vx_sim + k1) * dt * np.cos(pred_pos.theta + m1)
            l2 = (vx_sim + k1) * dt * np.sin(pred_pos.theta + m1)
            m2 = vw_sim * dt
        
            pred_pos.x += (k1 + k2) / 2
            pred_pos.y += (l1 + l2) / 2
            pred_pos.theta += m1
        
            t += dt  # 更新时间

        return pred_pos

#路径评估函数
    def evaluate_trajectory(self,vx_iter, vw_iter):
        pred_pos = self.predict_pos(vx_iter, vw_iter)

        heading_score = np.abs(np.arctan2(self.target.y - self.pos.y, self.target.x - self.pos.x) - np.arctan2(np.sin(pred_pos.theta), np.cos(pred_pos.theta))) / (2 * np.pi)

        target_score =  1 / (1 + np.sqrt(pow(pred_pos.x - self.target.x, 2) + pow(pred_pos.y - self.target.y, 2)))

        obs_distances = [np.sqrt(pow(pred_pos.x - obs.x, 2) + pow(pred_pos.y - obs.y, 2)) for obs in (self.obstacles + self.obstacles_predict)]
        min_dist = np.min(obs_distances)

        if min_dist < self.dangerous_dist:
            if min_dist > 100:
                dist_score = 1 - np.power(1 - (min_dist - 100) / (self.dangerous_dist - 100), 4)
            elif min_dist < 100:
                dist_score = float('-inf')
        else:
            dist_score = 1.0

        if np.sqrt(pow(pred_pos.x - self.target.x, 2) + pow(pred_pos.y - self.target.y, 2)) < 100:
            vel_score = 1.0 - vx_iter / self.vx_range.max
        else:
            vel_score = vx_iter / self.vx_range.max

        score = 0 * heading_score + 50 * dist_score + 10 * vel_score + 20 * target_score

        return score, vx_iter, vw_iter

    def navigate(self):
        #生成速度空间网格
        vx_min = np.maximum(self.vx_range.min, self.vel.vx + self.simt * self.dx_range.min)
        vx_max = np.minimum(self.vx_range.max, self.vel.vx + self.simt * self.dx_range.max)
        vw_min = np.maximum(self.vw_range.min, self.vel.vw + self.simt * self.dw_range.min)
        vw_max = np.minimum(self.vw_range.max, self.vel.vw + self.simt * self.dw_range.max)

        vxs = np.linspace(vx_min, vx_max, self.res)
        vws = np.linspace(vw_min, vw_max, self.res)

        vx_grid, vw_grid = np.meshgrid(vxs, vws)
        vx_flat = vx_grid.flatten()
        vw_flat = vw_grid.flatten()

        final_score = float('-inf')
        slct_vx = 0
        slct_vw = 0
        pred_pos = None
        with ThreadPoolExecutor(max_workers=4) as executor:
            futures = [executor.submit(self.evaluate_trajectory, vx_iter, vw_iter) for vx_iter, vw_iter in zip(vx_flat, vw_flat)]
            for future in concurrent.futures.as_completed(futures):
                score, vx_iter, vw_iter = future.result()
                if score > final_score:
                    final_score = score
                    slct_vx = vx_iter
                    slct_vw = vw_iter
                    pred_pos = self.predict_pos(slct_vx, slct_vw)
        self.vel = Vel(slct_vx, slct_vw)
        return slct_vx, slct_vw, pred_pos.x, pred_pos.y

    def set_target(self, target_x, target_y):
        self.target = Pos(target_x, target_y, 0.0)

       