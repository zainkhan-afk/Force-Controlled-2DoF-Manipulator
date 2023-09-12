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
arm = Arm(sim, position = (6.0, 6.0))
ground = Ground(sim)

sim.AddEntity(arm)
sim.AddEntity(ground)

x = 0
y = 0
ang = 0

force = np.array([[0, -1]]).T

while True:
	ret = sim.Step()
	y = 1.5*np.sin(ang)

	# force = np.array([x, y])
	position = np.array([x, y])

	arm.GoTo(position)
	# arm.ApplyForce(force)

	ang += 0.001

	if not ret:
		sys.exit()