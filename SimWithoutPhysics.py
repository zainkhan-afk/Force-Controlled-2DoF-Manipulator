import numpy as  np

from Robot import Dummy
import cv2
from utils import *


def DrawRobot(canvas, arm, scale = 100):
	H, W, _ = canvas.shape
	arm_width = 50

	l1 = arm.l1*scale
	l2 = arm.l2*scale

	T1 = GetTransformationMatrix(arm.state.theta[0], W/2, H/2)
	T2 = GetTransformationMatrix(arm.state.theta[1], 0, l1)

	link_1_end = np.array([[0, l1, 1]]).T
	link_2_end = np.array([[0, l2, 1]]).T

	link_1_end = (T1@link_1_end).astype("int")
	link_2_end = (T1@T2@link_2_end).astype("int")

	cv2.line(canvas, (W//2, H//2), (link_1_end[0, 0], link_1_end[1, 0]), (255, 0, 0), 3)
	cv2.line(canvas, (link_1_end[0, 0], link_1_end[1, 0]), (link_2_end[0, 0], link_2_end[1, 0]), (0, 0, 255), 3)


SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480
t = 0
T = 100 # Seconds

l1 = 1
l2 = 1

m1 = 0.4
m2 = 0.4

arm = Dummy(l1, l2, m1, m2)


force = np.array([[-gravity, 0]]).T

reached_home = False
while True:
	canvas = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH, 3)).astype("uint8")

	if not reached_home:
		reached_home = arm.HomePosition()
		print("Goint to home position.")

	else:
		arm.ApplyForce(force)

	DrawRobot(canvas, arm, scale = 100)

	cv2.imshow("canvas", canvas)

	k = cv2.waitKey(30)

	if k == ord("q"):
		break