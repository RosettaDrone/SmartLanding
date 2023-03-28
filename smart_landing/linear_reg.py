from collections import deque
from sklearn.linear_model import LinearRegression

def linearRegressionTest():
	# Linear regressor to smooth last velocity measurements and to obtain current velocity using prediction.
	# We use a prediction of the current velocity based on measurments that are received with a delay caused by the visual pose estimation algorithm.
	# If the drone is moving at constant velocity, the prediction will return the average of the last measurements
	# If the drone is accelerating, the prediction will estimate the current velocity using linear regression.

	# Add some observations (received with delay)
	reg = LinearReg(3)
	reg.add(-4, 0) # t = -4
	reg.add(-3, 0) # t = -3
	reg.add(-2, 0) # ...
	reg.add(-1, 1)

	currentVel = reg.predict(0)

	print("currentVel: " + str(currentVel))

class LinearReg:
	# buffer_size: number of measurements to consider for the linear regression. Old values will be removed
	def __init__(self, buffer_size):
		self.xValues = deque(maxlen=buffer_size)
		self.yValues = deque(maxlen=buffer_size)
		self.t = 0

	def add(self, x, y):
		self.xValues.append([x])
		self.yValues.append(y)
	
	def getReg(self):
		return LinearRegression().fit(self.xValues, self.yValues)

	def predict(self, t):
		print("RegValues: ", self.xValues, self.yValues)
		reg = self.getReg()
		y = reg.predict([[t]])
		return y[0]
	
	# Estimate X given Y (the inverse)
	def getX(self, y):
		reg = self.getReg()
		m = reg.coef_
		b = reg.intercept_
		return (y - b) / m