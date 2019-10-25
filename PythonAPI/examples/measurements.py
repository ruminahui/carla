import carla



class PlayerMeasurements():
	""" Class to patch the player measurements structure on the newest version of carla (0.9.6) """
	def __init__(self):
		self.transform = carla.Transform()
		self.acceleration = carla.Vector3D()
		self.forward_speed = 0
		self.collision_vehicles = 0
		self.collision_pedestrians = 0
		self.collision_other = 0
		self.intersection_otherlane = 0
		self.intersection_offroad = 0
		self.autopilot_control = carla.VehicleControl()


class Measurements():
	""" Class to patch the measurements structure on the newest version of carla (0.9.6) """
	def __init__(self):
		self.player_measurements = PlayerMeasurements()
		# Do not pack any information about other agents for now
		self.non_player_agents = []



	def update_measurements(self, image, player):
		""" Populates the information represented by this class by retrieving information from the image and the player """
		self.player_measurements.forward_speed = player.get_forward_speed() * 0.01 
		self.player_measurements.transform = player.get_transform()
		self.player_measurements.acceleration = player.get_acceleration()







