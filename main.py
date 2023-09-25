import sys
import numpy as  np

from simulation import Simulation
from Robot import Arm
from ground import Ground

from utils import *



PPM = 50.0  # pixels per meter
TARGET_FPS = 60
# TIME_STEP = 0.001
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

sim = Simulation(width = SCREEN_WIDTH, height = SCREEN_HEIGHT, delta_T = TIME_STEP, PPM = PPM, FPS = TARGET_FPS)
ground = Ground(sim)
arm = Arm(sim, ground, position = (6.0, 6.0))

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

x = 0
y = 0
ang = 0

force = np.array([[0, 0]]).T


while True:
	y = 1.5*np.sin(ang)

	# force = np.array([x, y])
	position = np.array([[x, y]]).T

	# arm.GoTo(position)
	arm.ApplyForce(force)
	# arm.GoToAngle(np.pi/2, np.pi/2)
	# EE_Pos = arm.GetEEPos()


	# print(position, EE_Pos)
	# EE_x = EE_Pos[0, 0]
	# EE_y = EE_Pos[1, 0]

	# if EE_x>0.1:
	# 	print("Reversing", force)
	# 	force[0, 0] = -10

	# if EE_x<-0.1:
	# 	print("UnReversing", force)
	# 	force[0, 0] = 1

	# print(EE_x, EE_y)

	ang += 0.0001

	ret = sim.Step()
	if not ret:
		sys.exit()