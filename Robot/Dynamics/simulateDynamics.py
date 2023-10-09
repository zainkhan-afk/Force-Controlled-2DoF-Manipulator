import numpy as np

class SimulateDynamics:
	def __init__(self, robotDynamicsModel):
		self.robotDynamicsModel = robotDynamicsModel

	def GoToNextStateFD(self, force, J, current_state):
		theta_double_dot, _ = self.robotDynamicsModel.ForwardDynamics(force, J, current_state)

		new_state = current_state.UpdateUsingAcceleration(np.array([theta_double_dot[0, 0], theta_double_dot[1, 0]]))

		return new_state