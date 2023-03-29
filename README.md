# SmartLanding

Drone precision landing algorithms and framework for testing in a simulator.

This algorithm uses only relative positions and velocities obtained via computer vision.
We asume that absolute positions and velocities are unknown. Furthermore we don't want to depend on GPS and sensors, but solely on vision.

We implemented two chasing algorithms, where a drone with a camera tracks and chases another object like a fiducial marker placed on the ground or on another moving vehicle.

The only input of the algoritm is the position of the landing target relative to the drone's camera, which is then used to compute the relative velocity (target - drone).

The algorithm supports high "observation latency" (the position of the target is received with a delay) and supports high "action latency" (the response time of the drone to change its velocity).

We implemented two chasing algorithms:

## Linear chasing algorithm

- We compute the relative velocity of the target based on delayed observations of its relative position.
- We use linear regression to predict the current relative position and relative velocity of the target.
- We adjust the drone's velocity so that:
	1) the drone moves with the same velocity vector as the target (their relative velocity is zero)
	2) but we also add a velocity vector to move the drone in the direction of the predicted position of target
- We wait until the drone's velocity change transient finishes (the drone finished accelerating) and then compute a new velocity adjustment. See detailed comments in the code (search for "WAIT_TRANSIENT:").

## PID controlled chasing algorithm

- Similar to previous algorithm, but instead of changing the velocity of the drone in exact amounts of velocity differences computed during a "study phase" (while the drone is not accelerating), here we use a PID controller to adjust the velocity to the desired relative velocity continuously.

## Comparison

The "PID controlled chasing algorithm" offers a smoother motion, but uses "magic" parameters that have to be fine tuned via trial and error, while the "Linear chasing algorithm" offers a more transparent and exact approach.

## Simulator

The code is provided as a library that can be included in [Vision Landing](https://github.com/kripper/vision-landing-2) or tested visually as a stand alone.

On the image we see the target (in red) being chased by the drone (in blue) and the drone's velocity adjustment vectors.
In this test we simulated an observation latency and action latency of 1 [s] both, ie. the drone's velocity is adjusted with a total delay of 2 [s].

![image](https://user-images.githubusercontent.com/1479804/228393293-d3638265-8aa2-4070-ba8f-86cea76d3262.png)

