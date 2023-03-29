"""
The DroneContrller class simulates the motion of the drone and does prediction.
"""

from linear_reg import LinearReg
import numpy as np
from collections import deque
from simple_pid import PID

import matplotlib.pyplot as plt
from matplotlib.animation import ArtistAnimation

class DroneController:
	def __init__(self, usePID, cameraDelay, actionDelay):
		self.usePID = usePID
		self.usePrediction = True

		self.x = np.array([0.0, 0.0])
		self.droneVel = np.array([0.0, 0.0]) # Current drone velocity

		self.lastX = None

		size = 4 # Linear Regression observations. 4 is better then 3, because [0,0,1] generates too much error compared to [0,0,1,1]
		self.regX = LinearReg(size) # LR for the x-coord
		self.regY = LinearReg(size) # LR for the y-coord

		# Variables used for simulating gradual velocity change.
		self.additionalVel = np.array([0.0, 0.0]) # Velocity to be added
		self.setVelCount = 0 # We count cycles and make sure that self.additionalVel is completely added to self.droneVel
		self.actionDelay = actionDelay # Cycles required to finish velocity change
		self.totalDelay = cameraDelay + actionDelay

		if usePID:
			# TODO: Check other options, like: https://en.wikipedia.org/wiki/Smith_predictor
			kp = 1
			ki = 0.1
			kd = 0.05

			self.pidX = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=0.0)
			self.pidY = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=0.0)

		self.maxApproachingVel = 3 # Max velocity in [m/s] used to approach the target (additionaly to the velocity of the moving target)

		self.firstTime = True
		self.waitTransient = 0

		self.simulator = None # Only used for debugging in simulator

	def process(self, currentTime, obsT, obsTargetX):
		obsVel = self.recPos(obsT, obsTargetX) # Observed relative velocity

		sim = self.simulator

		if not self.firstTime:
			dt = currentTime - obsT # Time since last obs

			# NOTE: curPos and curVel are current *relative* position and velocities (target - camera)

			if self.usePrediction:
				if True:
					# Use last observed velocity instead of using a linear regression, because it predicts with big error when we start moving.
					# TODO: Use prediction, unless last observed velocities don't fit on a line
					curVel = obsVel
				else:
					curVel = self.getVel(currentTime) # current estimated velocity

				curPos = obsTargetX + (obsVel + curVel) / 2 * dt # current estimated position = last-obs-position + avg velocity (between last observed velocity and current velocity) + avg velocity (between current velocity and future velocity)

			else:
				# Use last observed velocity and position
				curVel = obsVel
				curPos = obsTargetX

			# Now, we will compute the required relative velocity (desiredVel) so that:
			# 1) the drone moves with the same velocity than the target (curVel)
			# 2) plus, the drone moves in direction to the target (approachingVel)

			# curPos is the target position relative to the drone = vector pointing from the drone to the target
			targetDistance = np.linalg.norm(curPos)

			# Approaching velocity shouldn't be greater than "targetDistance / totalDelay" (distance / time) to avoid overshooting. In totalDelay seconds we will reset or adjust the velocity again.
			approachingVelScalar = min(targetDistance / self.totalDelay, self.maxApproachingVel)

			# (curPos / targetDistance) is the normal vector in direction to the target
			approachingVel = (curPos / targetDistance) * approachingVelScalar

			# This is the desired relative velocity
			desiredVel = curVel + approachingVel

			drawDroneVelocityVector = False
			if self.usePID:
				# We won't measure the drone's absolute velocity, but only the target relative velocity from the drone to the target.
				# All we do is change the output velocities to minimize the error.
				error = -desiredVel
				deltaVel = np.array([pidX(error[0]), pidY(error[1])]) * 0.1 # TODO: 0.1 is a magical number (trial and error)
				self.addVel(deltaVel)

			else:
				# WAIT_TRANSIENT:
				# When we change the drone velocity it doesn't change immediately. The drone accelerates during a short time (called "transient") until it reaches the desired velocity.
				# During the transient, we don't want to compute and change the velocity again, since the computation will not consider the velocity change in progress.
				# Otherwise, we would be computing and adding **the same** velocity increment (deltaVel) multiple times until the drone's velocity actually changed.
				# Thus, each time we change the velocity, we will wait self.totalDelay [s] before computing and adjusting the velocity again.
				# We also want to wait until velocity **of the drone** is constant again to make sure that the measured and predicted relative velocity of the target is not affected by the drone's acceleration.
				if obsT > self.waitTransient:
					# By incrementing the current velocity in desiredVel (= curVel + approachingVel), the drone will
					# move with the same velocity as the target (curVel) and their relative velocity will be canceled
					# except for approachingVel which moves the drone to the target at a controlled velocity (approachingVelScalar)
					deltaVel = desiredVel

					self.addVel(deltaVel)
					drawDroneVelocityVector = True
					self.waitTransient = obsT + self.totalDelay # Wait until velocity reached in observations
				else:
					deltaVel = np.array([0.0, 0.0])

			if sim is not None:
				
				sim.newFrame(currentTime)

				if sim.drawPredictions:
					if not sim.renderAbsolute:
						sim.obsDroneX = np.array([0,0])

					absTargetX = sim.obsDroneX + curPos

					# Estimated current target position
					sim.renderMarker(absTargetX, "+", "green")

					# Estimated velocity
					sim.renderArrow(absTargetX[0], absTargetX[1], curVel[0], curVel[1], "blue")

				# Drone velocity
				if drawDroneVelocityVector:
					#print("desiredVel: ", desiredVel)
					sim.renderArrow(self.simulator.obsDroneX[0], self.simulator.obsDroneX[1], desiredVel[0], desiredVel[1], 'red')

				absObsTargetX = sim.obsDroneX + obsTargetX # Absoulte observed target position
				sim.renderTarget(absObsTargetX)

				if sim.renderAbsolute:
					sim.renderDrone(sim.obsDroneX)

				sim.pushFrame()

		else:
			self.firstTime = False
			deltaVel = np.array([0.0, 0.0])

		return deltaVel

	def recPos(self, t, x):
		#print("obs x (t=" + str(t) + "): " + str(x))
		if self.lastX is not None:
			v = x - self.lastX

			self.regX.add(t, v[0])
			self.regY.add(t, v[1])

			#print("vel (t=" + str(t) + "): " + str(v))
		else:
			v = np.array([0, 0])

		self.lastX = x.copy()

		return v
	
	def getVel(self, t):
		return np.array([
			self.regX.predict(t),
			self.regY.predict(t)
		])
	
	def addVel(self, acc):
		self.setVelCount = self.actionDelay
		self.additionalVel += acc
	
	# Used only for testing
	def move(self):
		self.x += self.droneVel

		# Gradually velocity change
		self.setVelCount = self.setVelCount - 1
		if self.setVelCount <= 0:
			# Finish velocity change
			delta = self.additionalVel
		else:
			delta = self.additionalVel * 0.5

		self.droneVel += delta
		self.additionalVel -= delta

# Class representing the Landing Target
class Target:
	def __init__(self):
		self.x = np.array([4.0, 5.0]) # Current target position relative to camera
		self.v = np.array([1.0, -0.2]) # Current target velocity relative to camera
		self.i = 0

	# Used only for testing
	def move(self):
		if True:
			if self.i < 10:
				# None
				dummy = 0
			elif self.i < 20:
				self.v -= np.array([-0.1, 0.3])
			elif self.i < 30:
				self.v += np.array([-0.4, -0.1])
			elif self.i < 40:
				self.v = np.array([0,0])

		self.i = self.i + 1

		self.x += self.v

# Simple motion simulator to test the landing algorithm
class Simulator:
	def __init__(self, bufferSize, slowMotion):
		self.buffer = deque(maxlen=bufferSize) # Visual buffer, used to simulate the visual computing delay

		self.overlay = False
		self.drawPredictions = False

		self.frames = [] # Animation frames
		self.fig, self.ax = plt.subplots()

		if slowMotion:
			# Micro-analysis
			self.interval = 4000
		else:
			# Macro-analysis
			self.interval = 100

	# Add a new observation to the visual buffer (to simulate the computer vision latency)
	def pushVisualBuffer(self, target, drone):
		self.buffer.append([
			target.x.copy(),
			drone.x.copy()
		])
	
	def popVisualBuffer(self):
		return self.buffer.popleft()

	def render(self, artObj):
		self.line.append(artObj)

	def newFrame(self, t):
		self.line = self.ax.plot([0,0], [0,0]) # HACK: Dummy to get an artistic object. See: https://github.com/matplotlib/matplotlib/issues/18381
		if not self.overlay:
			self.render(self.ax.text(5, 5, "t=" + str(t), fontsize=12))
	
	def renderMarker(self, x, marker, color):
		self.render(self.ax.scatter([x[0]], [x[1]], marker=marker, color=color))
	
	def renderDrone(self, x):
		self.renderMarker(x, "*", "blue")

	def renderTarget(self, x):
		self.renderMarker(x, "o", "red")

	def renderArrow(self, x1, y1, x2, y2, color):
		self.render(self.ax.arrow(x1, y1, x2, y2, head_width=0.2, head_length=0.3, alpha=0.5, color=color))

	def showAnimation(self):
		plt.xlabel('x')
		plt.ylabel('y')

		# BUG: Axis are not drawn on origin when using zoom.
		plt.axhline(y=0, color='black', linewidth=0.1)
		plt.axvline(x=0, color='black', linewidth=0.1)

		#plt.xlim(-10, 20)
		#plt.ylim(-10, 20)

		animation = ArtistAnimation(self.fig, self.frames, interval=self.interval, blit=True)

		plt.show()

	def pushFrame(self):
		if not self.overlay:
			self.frames.append(self.line)