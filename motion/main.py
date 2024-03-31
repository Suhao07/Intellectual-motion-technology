from vision import Vision
from action import Action
from debug import Debugger
from prm import PRM
from astar import AStar
from rrtstar import RRTStar
# from astarplus import AStarplus
from astarthreading import AStarthreading 
import time

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    planner = AStar()
    # planner =AStarthreading ()
    # planner = RRTStar()
    while True:
        # 1. path planning & velocity planning
        start_x, start_y = vision.my_robot.x, vision.my_robot.y
        goal_x, goal_y = -2400, -1500
        path_x, path_y, road_map, sample_x, sample_y = planner.plan(vision=vision, 
            start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)

        # 2. send command
        action.sendCommand(vx=0, vy=0, vw=0)

        # 3. draw debug msg
        debugger.draw_all(sample_x, sample_y, road_map, path_x, path_y)
        
        time.sleep(0.1)

# TODO::
#     A*改进；
#     深度学习/强化学习路径规划方法；
#     加入避障规划；