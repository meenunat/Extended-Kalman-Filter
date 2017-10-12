# Extended Kalman Filter Project

## Objective

In this project, kalman filter is used to estimate the state of a moving object of interest with noisy lidar and radar measurements. RMSE values are also calculated. 

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.

The main program can be built and run by doing the following from the project top directory.

	1. mkdir build
	2. cd build
	3. cmake .. 
	4. make
	5. ./ExtendedKF

## Data

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

## Kalman Filter Algorithm Steps

The Kalman Filter algorithm will go through the following steps:

** first measurement** - the filter will receive initial measurements of the object's position relative to the car. These measurements will come from a radar or lidar sensor.

** initialize state and covariance matrices** - the filter will initialize the object's position based on the first measurement.Then the car will receive another sensor measurement after a time period Δt.
	
** predict** - the algorithm will predict where the object will be after time Δt. One basic way to predict the object location after Δt is to assume the object's velocity is constant; thus the object will have moved velocity * Δt. 
	
** update** - the filter compares the "predicted" location with what the sensor measurement says. The predicted location and the measured location are combined to give an updated location. The Kalman filter will put more weight on either the predicted location or the measured location depending on the uncertainty of each value then the car will receive another sensor measurement after a time period Δt. The algorithm then does another predict and update step.


## Definition of Lidar variables: <a name="lidar"></a>

- z is the measurement vector. For a lidar sensor, the z vector contains the position−x and position−y measurements.

- H is the matrix that projects your belief about the object's current state into the measurement space of the sensor. For lidar, this is a fancy way of saying that we discard velocity information from the state variable since the lidar sensor only measures position: The state vector x contains information about [p​x​​,p​y​​,v​x​​,v​y​​] whereas the z vector will only contain [px,py]. Multiplying Hx allows us to compare x, our belief, with z, the sensor measurement.

## Definition of Radar variables: <a name="radar"></a>


- The range, (ρ), is the distance to the pedestrian. The range is basically the magnitude of the position vector ρ which can be defined as ρ=sqrt(p​x​2​​+p​y​2​​).

- φ=atan(p​y​​/p​x​​). Note that φ is referenced counter-clockwise from the x-axis, so φ from the video clip above in that situation would actually be negative.

- The range rate, ​ρ​˙​​, is the projection of the velocity, v, onto the line, L.
	
### Normalizing Angles
	
In C++, atan2() returns values between -pi and pi. When calculating phi in y = z - h(x) for radar measurements, the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi. The Kalman filter is expecting small angle values between the range -pi and pi.