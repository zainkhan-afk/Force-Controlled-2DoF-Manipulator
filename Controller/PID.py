import numpy as np
from Robot.Dynamics.simulateDynamics import SimulateDynamics

class PID:
	def __init__(self, dynamicsModel, P = 300, I = 0, D = 50):
		self.dynamicsSimulator = SimulateDynamics(dynamicsModel)
		self.P = P;
		self.I = I;
		self.D = D;

		# self.error = np.array([0, 0])
		self.prev_error = np.array([0, 0]).astype("float32")
		self.error_sum = np.array([0, 0]).astype("float32")

		self.prev_goal_pos = None

	def Solve(self, current_state, J, ee_pos, goal_pos):
		# if self.prev_goal_pos is None:
		# 	self.prev_goal_pos = goal_pos

		# if (goal_pos != self.prev_goal_pos).all():
		# 	self.error_sum = np.array([0, 0]).astype("float32")

		error = goal_pos - ee_pos.ravel()
		force = self.P*error + self.D*(self.prev_error - error) + self.I*self.error_sum

		new_state = self.dynamicsSimulator.GoToNextStateFD(force, J, current_state)

		self.prev_error = error
		self.error_sum += error


		return new_state