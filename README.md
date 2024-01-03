# hybrid_pp
From package '[hybrid_pp](https://github.com/ISC-Project-Phoenix/hybrid_pp)'
# File
`./src/hybrid_pp.cpp`

## Summary

Path tracking implementation of the pure pursuit algorithm used to command velocity and ackerman angle to navigate to calculated points along a selected path. The hybrid section
will come into play with the use of the lidar and a TBD obstacle avoidance algorithm. This is a statless implementation and works
only off its subscriptions.

### Publishes

- `/nav_ack_vel`:  The calculated ackerman angle and velocity to be set to phoenix.
- `visualization_marker`: The look ahead distance, path arc prediction, and interpolated path to be plotted on rviz for live visualization of the path predictions.

### Subscribes

- `/odom`: Odometry, this is used to get the linear velocity to determine a look ahead distance.
- `/path`: This is the path that is being supplied from the obj_planner node, should be a list of midpoints between respective left and right cones.

## Params

- `min_look_ahead_distance`: The minimum look ahead distance to intersect the path and obtain a path point that is kinematically possible for phoenix.
- `max_look_ahead_distance`: The maximum look ahead distance to intersect the path and obtain a path point that is within the max range of the camera.
- `k_dd`: A constant multiplier used to scale the speed in the calculation for look ahead distance.
- `rear_axle_frame`: A string used for the odom to rear_axle transformation.
- `max_speed`:  Max possible speed of phoenix in meters per second.
- `wheel_base`: The wheelbase of phoenix in meters.
- `gravity_constant`: The constant acceleration due to gravity.
- `debug`: Debug flag to determine if we want to publish the visualization markers.