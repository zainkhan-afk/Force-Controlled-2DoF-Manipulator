import sys
import numpy as  np

from simulation import Simulation
from Robot import Arm
from ground import Ground

from utils import *
from state import State
from Controller import PID
from path import Path


PPM = 50.0  # pixels per meter
TARGET_FPS = 60
# TIME_STEP = 0.001
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480
ARM_POSITION = (6.0, 6.0)

sim = Simulation(width = SCREEN_WIDTH, height = SCREEN_HEIGHT, delta_T = TIME_STEP, PPM = PPM, FPS = TARGET_FPS)
ground = Ground(sim)
arm = Arm(sim, ground, position = ARM_POSITION)

pid_controller = PID(arm.dynamicsModel, P = 1000, I = 10, D = 100)

path = Path(size = 500, transform = ARM_POSITION)


ang = 0
randius = 0.5
for i in range(250):
	point = np.array([randius*np.cos(ang), -1 + randius*np.sin(ang)])
	path.AddPoint(point)
	ang += 0.1
	randius += 0.001

sim.AddEntity(arm)
sim.AddEntity(ground)
sim.AddEntity(path)

path.MakePathRenderPts(sim.renderer.screen, sim.PPM)

home_pos_reached = False
print("Going to home position.")
while not home_pos_reached:
	ret = sim.Step()
	home_pos_reached = arm.HomePosition()

	if not ret:
		sys.exit()



arm.UpdateState()
arm.UpdateState()
print("Reached home position.")

desired_state = State(np.array([0, 0]), np.array([0, 0]), np.array([0, 0]))


while True:
	# goal_pos = np.array([0.5*np.cos(ang), -1 + 0.5*np.sin(ang)])
	# ee_pos = arm.GetEEPos()
	# force = pid_controller.CalculateForce(ee_pos, goal_pos)
	# force = arm.GetForce(desired_state)
	# print(force)
	# force = np.array([0, 0])
	# arm.ApplyForce(force)

	# goal_pos[1] = 0.5*np.sin(ang)

	# print(goal_pos)

	ret = sim.Step()
	if not ret:
		sys.exit()