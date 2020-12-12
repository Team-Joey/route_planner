import 	rospy

class FoodItem(object):
	'''
	This class is a model for each food item. 
	'''
	
	def __init__(self, x, y, label):
		self.x = x 			# x coordinate of the food item
		self.y = y			# y coordinate of the food item
		self.label = label		# label/ description of the food item

	def getLabel():
		return self.label

	def getCoordinates():
		return self.x, self.y

	def pick_up():
		'''
		'''
		print("Robot picked up ", label)
