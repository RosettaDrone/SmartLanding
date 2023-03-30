"""
Smart Landing
Web: https://github.com/kripper/SmartLanding
Author: Christopher Pereira (rosetta@imatronix.cl)

TODO:
- Linear chasing algorithm:
	- Compute the exact output velocity and timing required to reach the target perfectly (avoid requiring a second adjustment if the target's velocity is constant)
	- Autodetect when velocity change transient finishes instead of using a fixed delay
- PID controlled chasing algorithm:
	- Tune PID paramters
	- Limit output and prevent unstability
- General:
	- Re-enable non-constant velocity prediction. Temporarily disable because:
		- If the velocity changes from 0 to 1, the linear regression model will predict the next velocity is 2 (0,1,2,...). This is wrong when motion starts after lying in repose (0,1,1,1,...).
		- We could filter out outliers or detect if the data doesn't fit on a line and avoid doing a prediction (we could just use the last observed velocity in this case). On the other hand, it could be convenient to overreact during the initial motion to compensate the motion delay (?).
		- Anyway, also consider that further measurments will fix the initial overrated estimation.
"""

import numpy as np
import time

from smart_landing.drone_controller import DroneController, Target, Simulator

def motionTest():
	# Select algorithm: False = "Linear", True = "PID". See README for more info.
	usePID = False

	iterations = 100

	cameraDelay = 1 # Delay of observations = time between last observation and time when data is received
	actionDelay = 1 # Time required to reach a desired velocity

	slowMotion = False # Show in slow motion. TODO: Add buttons to move forward, backward.

	sim = Simulator(cameraDelay, slowMotion)
	sim.renderAbsolute = True # Render absolute positions/velocities instead of relative positions/velocities
	sim.drawPredictions = not sim.renderAbsolute # Predictions are rendered wrong in absolute frame
	sim.interactive = False # Refreshes the plot immediately. Usefull for testing the while system when used as a library.
	sim.overlay = False # Show all frames at the same time instead of an animation

	target = Target()

	drone = DroneController(usePID, cameraDelay, actionDelay)
	drone.simulator = sim # Attach simulator to DroneController

	# Fill visual buffer with observations (with the drone and the target lying in repose)
	for i in range(0, cameraDelay):
		sim.pushVisualBuffer(target, drone)

	for currentTime in range(0, iterations):
		# print("\n*** t = " + str(currentTime) + " ***")

		# Get delayed observation
		obsTargetX, obsDroneX = sim.popVisualBuffer()
		sim.obsDroneX = obsDroneX # Required for rendering absolute positions

		obsT = currentTime - cameraDelay # Time of observation
		
		drone.process(currentTime, obsT, obsTargetX)

		# Keep target position relative to the drone
		target.x -= drone.droneVel

		# Move the drone and target
		drone.move()
		target.move()

		sim.pushVisualBuffer(target, drone)

	sim.showAnimation()

#smart_landing.linear_reg.linearRegressionTest()
motionTest()