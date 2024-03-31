import socket
import sys
import time
import math
from zss_cmd_pb2 import Robots_Command, Robot_Command

class Action(object):
	def __init__(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.command_address = ('localhost', 50001)
		self.max_speed=3000
		self.max_acceleration = 3000  # 设置加速度上限 TODO
		self.current_vx = 0
		self.current_vy = 0
		self.current_vw = 0
	def sendCommand(self, vx=0, vy=0, vw=0,step_time=999):
     # 检查速度是否超过上限
		commands = Robots_Command()
		command = commands.command.add()
		command.robot_id = 0
		command.velocity_x = vx
		command.velocity_y = vy
		command.velocity_r = vw
		if abs(vx) > self.max_speed:
			command.velocity_x = self.max_speed if vx > 0 else -self.max_speed
		if abs(vy) > self.max_speed:
			command.velocity_y = self.max_speed if vy > 0 else -self.max_speed
		if abs(vw) > self.max_speed:
			command.velocity_r = self.max_speed if vw > 0 else -self.max_speed
   # 检查加速度是否超过上限
		ax = (vx - self.current_vx)/step_time
		aw = (vw - self.current_vw)/step_time
		acceleration = math.sqrt(ax**2 + aw**2)
		if acceleration > self.max_acceleration:
			scale = self.max_acceleration / acceleration
			vx = self.current_vx + ax * scale
			vw = self.current_vw + aw * scale
		# print(command.velocity_x)
		# print(command.velocity_r)
		command.kick = False
		command.power = 0
		command.dribbler_spin = False
		self.sock.sendto(commands.SerializeToString(), self.command_address)

if __name__ == '__main__':
	action_module = Action()
	while True:
		action_module.sendCommand(vx=100, vw=10)
		time.sleep(0.02)
