# SmartLanding

Drone precision landing algorithms and framework for testing in a simulator.

This algorithm uses only relative positions and velocities obtained via computer vision.
We asume absolute positions and velocities are unknown. Furthermore we don't want to depend on GPS and sensors, but only on vision.

We implemented two chasing algorithms, where a drone with a camera tracks and chases another object like a fiducial marker placed on the ground for precision landing.
The only input of the algoritm is the position of the landing target relative to the drone's camera, which is then used to compute the relative velocity (target - drone).
The algorithm supports high "observation latency" (the position of the target is received with a delay)
and supports high "action latency" (the response time of the drone to change its velocity).

We implemented two chasing algorithms:

## Linear chasing algorithm

- We compute the relative velocity of the target based on delayed observations of the relative position of the target.
- We use linear regression to predict the current relative position and relative velocity of the target.
- We compute the relative velocity difference required to:
	1) move the drone with the same velocity vector as the target to cancel their relative velocity
	2) plus a velocity vector to move the drone in the direction of the target
- We wait until the drone's velocity change transient finished (the drone finished accelerating) and then compute a new velocity adjustment. See detailed comments in the code (search for "WAIT_TRANSIENT:").

## PID controlled chasing algorithm

- Similar to previous algorithm, but instead of changing the velocity of the drone in exact amounts of velocity differences computed during a "study phase" (while the drone is not accelerating),
  we use a PID controller to adjust the velocity to the desired relative velocity all the time.

## Comparison

The "PID controlled chasing algorithm" offers a smoother motion, but uses "magic" parameters that have to be fine tuned via trial and error, while the "Linear chasing algorithm" offers a more transparent and exact apporach.
