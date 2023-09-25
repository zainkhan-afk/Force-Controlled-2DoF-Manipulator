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

force = np.array([[0, 0]]).T


while True:
	arm.ApplyForce(force)
	
	ret = sim.Step()
	if not ret:
		sys.exit()