import sys
import numpy as  np

from simulation import Simulation
from Robot import Arm
from ground import Ground

from utils import *
from state import State
from Controller import PID


PPM = 50.0  # pixels per meter
TARGET_FPS = 60
# TIME_STEP = 0.001
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

sim = Simulation(width = SCREEN_WIDTH, height = SCREEN_HEIGHT, delta_T = TIME_STEP, PPM = PPM, FPS = TARGET_FPS)
ground = Ground(sim)
arm = Arm(sim, ground, position = (6.0, 6.0))
pid_controller = PID(P = 300, I = 0, D = 50)

sim.AddEntity(arm)
sim.AddEntity(ground)

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

goal_pos = np.array([0, -1])

ang = 0

while True:
	goal_pos = np.array([0.5*np.cos(ang), -1 + 0.5*np.sin(ang)])
	ee_pos = arm.GetEEPos()
	force = pid_controller.CalculateForce(ee_pos, goal_pos)
	# force = arm.GetForce(desired_state)
	# print(force)
	arm.ApplyForce(force)

	# goal_pos[1] = 0.5*np.sin(ang)

	# print(goal_pos)

	ang += 0.001
	ret = sim.Step()
	if not ret:
		sys.exit()