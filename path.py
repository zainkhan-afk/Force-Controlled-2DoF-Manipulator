import pygame
import numpy as np

class Path:
	def __init__(self, size = 100, transform = None):
		self.path = []
		self.size = size
		self.color_line = (255, 0, 0)
		self.transform = transform
		self.renderPts = None

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
				pt = np.array([self.transform[0] +     self.path[i][0], self.transform[1] +     self.path[i][1]]) * PPM

			else:
				pt = np.array([    self.path[i][0],     self.path[i][1]]) * PPM

			pt = (pt[0], H - pt[1])

			self.renderPts.append(pt)

	def Render(self, screen, PPM):
		if self.renderPts is None:
			H = screen.get_height()
			for i in range(len(self.path) - 1):
				if self.transform is not None:
					pt1 = np.array([self.transform[0] +     self.path[i][0], self.transform[1] +     self.path[i][1]]) * PPM
					pt2 = np.array([self.transform[0] + self.path[i + 1][0], self.transform[1] + self.path[i + 1][1]]) * PPM

				else:
					pt1 = np.array([    self.path[i][0],     self.path[i][1]]) * PPM
					pt2 = np.array([self.path[i + 1][0], self.path[i + 1][1]]) * PPM

				pt1 = (pt1[0], H - pt1[1])
				pt2 = (pt2[0], H - pt2[1])

				pygame.draw.line(screen, self.color_line, pt1, pt2, 1)

		else:
			pygame.draw.aalines(screen, self.color_line, False, self.renderPts)