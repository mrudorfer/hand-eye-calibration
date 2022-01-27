# ROS package for hand-eye calibration

Use case: We have a camera that is fixed in world frame, and we want to calibrate it with respect to a robot.

Note: My dependency management is a mess right now, so things might not work out of the box.
Hoping to improve on that soon, please raise issues.

Package is developed with ROS noetic / Ubuntu 20.04.

### General

#### Rough Approach

- use aruco markers which are placed on table in view field of camera
- detect markers by camera, publish marker frame to `/tf`
- measure transform from robot base to marker and publish to `/tf`
- tf chain is complete and can compute transform from camera to robot
- camera pose might be slightly noisy, depending on marker detection in current frame
- occlusions can impair the marker detection, but robust to change of camera pose

#### More Robust/Precise Approach

- starting point as in rough approach, but fix marker to end effector of robot 
- move robot around to generate multiple measurements of camera pose wrt world frame
- compute average of all the collected poses; publish statically to `/tf`
- markers are not needed at later stage
- precision increased due to averaging
- requires re-calibration when camera moves

### Nodes

All nodes can be parametrised, please refer to the python files to see which parameters you can apply.

`rosrun calibration detect_markers.py`
connects to camera input topic and tries to detect an aruco board based on cv2.aruco package.
Specify the aruco board using the parameters; DICT type is currently hard coded to `DICT_4x4_100` but can easily be changed.
The recognised marker frame is published as parent of camera frame to tf.
Image with drawn marker axes is output for debug purposes.

`rosrun calibration robot_marker_tf.py`
publishes a static transform from robot end effector to marker board (once).
The transform is hard-coded in the file and needs to be adjusted to your setup.

`rosrun calibration pose_recorder.py`
is used to record camera poses in world frame, assuming marker detection is active.
Everytime you press enter, a pose is recorded (make sure robot stands still and all markers are recognised well).
After recording, the poses are averaged and mean errors are displayed.
I recorded 30 poses from different view points and got following result:
```
directly averaging all source-->target transforms yields the following:
computing average over 30 poses
t: [0.3186263  0.33035292 0.79509764], q: [ 0.21645533 -0.85911871  0.45153532  0.10572597]
translational errors: avg 0.008974828606934286, std 0.00344945356059908
angular errors (degrees): avg 0.8986374893758822, std 0.3530813712449838
saved recorded and processed transforms to /home/rudorfem/ros/recorded_poses.npy
```
The node also saves other transformations, I thought it might be useful to perform some kind of optimisation
which assumes that the marker to end-effector transform is static but unknown.
This would allow to perform the calibration when measuring this transform is hard or impossible.
However, the optimisation is not implemented yet as averaging poses seemed to give reasonable results.


Finally, `rosrun calibration world_camera_tf.py`
publishes the world/camera transform repeatedly.
You need to adjust the hard-coded transform to the output of `pose_recorder`.