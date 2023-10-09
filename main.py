import sys
import numpy as  np

from simulation import Simulation
from Robot import Arm
from ground import Ground

from utils import *
from state import State
from Controller import PID
from path import Path

import matplotlib.pyplot as plt


PPM = 50.0  # pixels per meter
TARGET_FPS = 60
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480
ARM_POSITION = (6.0, 6.0)

sim = Simulation(width = SCREEN_WIDTH, height = SCREEN_HEIGHT, delta_T = TIME_STEP, PPM = PPM, FPS = TARGET_FPS)
ground = Ground(sim)
arm = Arm(sim, ground, position = ARM_POSITION)

pid_controller = PID(arm.dynamicsModel, P = 250, I = 0.25, D = 150)

path = Path(size = 500, transform = ARM_POSITION)


ang = 0
randius = 0.5

num_pts = 500
for i in range(num_pts):
	point = np.array([i/(num_pts/2) - 1, -1 + randius*np.sin(ang)])
	# point = np.array([i/(num_pts/2) - 1, -1])
	path.AddPoint(point)
	ang += 0.1
	randius -= 0.002

# point = np.array([ 1, 1])
# path.AddPoint(point)

# point = np.array([ -1, 1])
# path.AddPoint(point)

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

step = 0
new_thetas = []
current_thetas = []
while True:
	arm.UpdateState()
	ee_pos = arm.GetEEPos()
	path.UpdateGoalPoint(ee_pos, thresh = 0.01)
	goal_pos = path.GetCurrentGoalPoint()

	current_state = arm.GetState()
	J = arm.GetJacobian()
	
	
	new_state = pid_controller.Solve(current_state, J, ee_pos, goal_pos)
	arm.ApplyState(new_state)

	# print("Current: ", current_state.theta[0]*180/np.pi, current_state.theta[1]*180/np.pi)
	# print("New: ", new_state.theta[0]*180/np.pi, new_state.theta[1]*180/np.pi)

	# current_thetas.append(current_state.theta)
	# new_thetas.append(new_state.theta)

	# if step>10000:
	# 	break


	ret = sim.Step()
	step += 1
	if not ret:
		sys.exit()

new_thetas = np.array(new_thetas)
current_thetas = np.array(current_thetas)

plt.figure()
plt.title("Theta 1 Graph")
plt.plot(current_thetas[:, 0], label = "Theta 1 Current")
plt.plot(new_thetas[:, 0], label = "Theta 1 New")
plt.legend()

plt.figure()
plt.title("Theta 2 Graph")
plt.plot(current_thetas[:, 1], label = "Theta 2 Current")
plt.plot(new_thetas[:, 1], label = "Theta 2 New")
plt.legend()
plt.show()