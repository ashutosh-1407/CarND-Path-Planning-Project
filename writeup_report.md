### Writeup / README

### Rubric points

### 1. The code compiles correctly. 
The code compiles successfully without any errors.

### 2. The car is able to drive at least 4.32 miles without incident..
I tried multiple times and the car was able to drive more than this without any incident.

### 3. The car drives according to the speed limit.
The speed limit is respected.

### 4. Max Acceleration and Jerk are not Exceeded.
The car was accelerating within the limits.

### 5. Car does not have collisions.
No collision was detected for longer durations of autonomous driving.

### 6. The car stays in its lane, except for the time between changing lanes.
The car stays in lane in all the times except for when its changing lanes.

### 7. The car is able to change lanes
The car is able to change the lanes successfully and drive smoothly.

### Reflection

I started with the basic lane following strategy provided in the Project Q&A video and then tried to do lane change maneuver whenever the vehicle in front of us is too close i.e. within 30m of range. Below are the steps:

-> I initially used the previous_path_waypoints to use as a starting point and then used 3 points(each 30m apart) to store in a vector and fit a spline to these points for creating a trajectory.  
-> Before fitting a spline to the points, I transformed the points to vehicle coordinate such that the current yaw_angle is 0.
-> After this, based on the previous path size, I appended waypoints so as to create a total of 50 points in a spline. Here, used the inverse transform of the one used earlier, for transforming the waypoints back to world coordinates.
-> Now, I checked if there is a vehicle in front of us i.e. within 30m of range by grabbing d and s coordinates from the sensor fusion data and checking if they are in close range.
-> If there is a vehicle in front of us, I tried changing lanes in the order "lane change left" and "lane change right" respectively.
-> For doing LCL and LCR, I checked if there is enought gap in between the cars in the intended lane. I checked for a minimum distance of 25m behind our car and a minimum distance of 25m ahead of the car in the intended lane. If the distance threshold is met, then the lane change is safe.
-> I have first checked for lane change left and if not possible, then check for lane change right. Finally, decreasing the speed for that cycle.
-> If the vehicle is not too close, then increasing the acceleration.