from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody)
from Box2D import b2Filter
import pygame

class Base:
	def __init__(self, sim_handle, position, group_index):
		self.width  = 0.05
		self.height = 0.05
		self.body = sim_handle.world.CreateStaticBody(
						    position = position,
						    shapes=polygonShape(box=(self.width, self.height)),
						)
		box = self.body.CreatePolygonFixture(box=(self.width, self.height), density=2.5, friction=0.3, filter = b2Filter(groupIndex=group_index))
		print("Fixed Base:", self.body.mass)

		self.color = (0, 0, 255)


	def Render(self, screen, PPM):
		H = screen.get_height()
		for fixture in self.body.fixtures:
			shape = fixture.shape
			vertices = [(self.body.transform * v) * PPM for v in shape.vertices]
			vertices = [(v[0], H - v[1]) for v in vertices]
			pygame.draw.polygon(screen, self.color, vertices)