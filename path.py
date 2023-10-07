import pygame

class Path:
	def __init__(self, size = 100):
		self.path = []
		self.size = size
		self.color_line = (255, 0, 0)

	def AddPoint(self, point):
		self.path.append(point)

		while len(self.path) > self.size:
			self.path.reverse()
			self.path.pop()
			self.path.reverse()

	def ClearPath(self):
		self.path = []

	def Render(self, screen, PPM):
		for i in range(len(self.path) - 1):
			pt1 = self.path[i] * PPM
			pt2 = self.path[i + 1] * PPM

			pygame.draw.line(screen, self.color_line, pt1, pt2, 1)