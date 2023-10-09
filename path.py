import pygame
import numpy as np
from utils import AlmostEqual

class Path:
	def __init__(self, size = 100, transform = None):
		self.path = []
		self.size = size
		self.color_line = (255, 0, 0)
		self.transform = transform
		self.current_path_point_index = 0
		self.renderPts = None

	def GetCurrentGoalPoint(self):
		if len(self.path)>0:
			return self.path[self.current_path_point_index]
		return None

	def UpdateGoalPoint(self, ee_pos, thresh = 0.01):
		if self.current_path_point_index >= (len(self.path) - 1):
			return

		goal_pt = self.GetCurrentGoalPoint()
		if AlmostEqual(goal_pt.reshape(2, 1), ee_pos.reshape(2, 1), thresh = thresh):
			self.current_path_point_index += 1
			if self.current_path_point_index >= len(self.path):
				self.current_path_point_index = len(self.path) - 1

			print(f"Path point: {self.current_path_point_index + 1}/{len(self.path)}")

	def AddPoint(self, point):
		self.path.append(point)

		while len(self.path) > self.size:
			self.path.reverse()
			self.path.pop()
			self.path.reverse()

	def ClearPath(self):
		self.path = []

	def MakePathRenderPts(self, screen, PPM):
		self.renderPts = []
		H = screen.get_height()
		for i in range(len(self.path)):
			if self.transform is not None:
				pt = np.array([self.transform[0] +     2*self.path[i][0], self.transform[1] +     2*self.path[i][1]]) * PPM

			else:
				pt = np.array([    2*self.path[i][0],     2*self.path[i][1]]) * PPM

			pt = (pt[0], H - pt[1])

			self.renderPts.append(pt)

	def Render(self, screen, PPM):
		if self.renderPts is None:
			H = screen.get_height()
			for i in range(len(self.path) - 1):
				if self.transform is not None:
					pt1 = np.array([self.transform[0] +     2*self.path[i][0], self.transform[1] +     2*self.path[i][1]]) * PPM
					pt2 = np.array([self.transform[0] + 2*self.path[i + 1][0], self.transform[1] + 2*self.path[i + 1][1]]) * PPM

				else:
					pt1 = np.array([    2*self.path[i][0],     2*self.path[i][1]]) * PPM
					pt2 = np.array([2*self.path[i + 1][0], 2*self.path[i + 1][1]]) * PPM

				pt1 = (pt1[0], H - pt1[1])
				pt2 = (pt2[0], H - pt2[1])

				pygame.draw.line(screen, self.color_line, pt1, pt2, 1)

		else:
			pygame.draw.aalines(screen, self.color_line, False, self.renderPts)