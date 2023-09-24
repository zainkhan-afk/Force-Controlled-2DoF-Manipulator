from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody)
import pygame
class Ground:
	def __init__(self, sim_handle):
		self.position = (0, 1)
		self.body = sim_handle.world.CreateStaticBody(
						    position=self.position,
						    shapes=polygonShape(box=(50, 1)),
						)

		self.color = (125, 125, 125)


	def Render(self, screen, PPM):
		H = screen.get_height()
		for fixture in self.body.fixtures:
			shape = fixture.shape
			vertices = [(self.body.transform * v) * PPM for v in shape.vertices]
			vertices = [(v[0], H - v[1]) for v in vertices]
			pygame.draw.polygon(screen, self.color, vertices)