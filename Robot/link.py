import pygame
from Box2D import b2Filter
from utils import *

class Link:
	def __init__(self, sim_handle, position, width, height, group_index, color = (200, 200, 255)):
		self.body = sim_handle.world.CreateDynamicBody(position=position, angle=0)
		self.height = height
		self.width = width
		box = self.body.CreatePolygonFixture(box=(width, height), density=7, friction=0.3, filter = b2Filter(groupIndex=group_index))
		print("Link:", self.body.mass)
		
		self.color = color

	def Render(self, screen, PPM):
		H = screen.get_height()
		for fixture in self.body.fixtures:
			shape = fixture.shape
			vertices = [(self.body.transform * v) * PPM for v in shape.vertices]
			vertices = [(v[0], H - v[1]) for v in vertices]
			pygame.draw.polygon(screen, self.color, vertices)

	def GetTransform(self):
		return GetTransformationMatrix(self.body.angle, self.body.position[0], self.body.position[1])