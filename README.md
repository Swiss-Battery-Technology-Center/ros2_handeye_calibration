# ROS2 hand-eye calibration
This is a minimal ROS2 port of the functionality in the easy_handeye calibration package. The original README can be found below.

## Usage
**Launch file**
```bash
ros2 launch hand_eye_calibration calibration.launch.py
```
Launch file arguments:
- **tracking_base_frame**: optical origin TF frame name
- **tracking_marker_frame**: marker TF frame name
- **robot_base_frame**: robot base TF frame name
- **robot_effector_frame**: end-effector TF frame name (or any frame ridigly connected to the tracking_marker_frame)
- **calibration_type**: either `eye-on-base` or `eye-in-hand` (see below)


**Required components**   
This node assumes that the TF between the `robot_base_frame` and the `robot_effector_frame` and the TF between the `tracking_base_frame` and `tracking_marker_frame` are being published. For AprilTag markers, the latter TF can be obtained for example by using a ros Apriltag detector node (e.g. [here](https://github.com/wep21/apriltag_ros/tree/ros2-port) or [here](https://github.com/christianrauch/apriltag_ros)). If the TFs are not available, the node will crash when you attempt to take a sample.


**Taking samples**
```bash
ros2 service call /hand_eye_calibration/capture_point std_srvs/srv/Trigger {}
```
For each `Trigger` service call, the ROS node will query the TF tree. When the number of samples is more then three, the node returns the current calibration estimate at every service call. All information is also printed to screen by the node. 

**Note**: Ideally, you should collect at least 15 samples, and check for convergence of the calibration estimate.    
**Note**: Please encure the robot is still when triggering each capture.


------------------------------------------------
<br/><br/>



# SNIPPETS FROM ORIGINAL README

## Use Cases

If you are unfamiliar with Tsai's hand-eye calibration [1], it can be used in two ways:

- **eye-in-hand** to compute the static transform between the reference frames of
  a robot's hand effector and that of a tracking system, e.g. the optical frame
  of an RGB camera used to track AR markers. In this case, the camera is
  mounted on the end-effector, and you place the visual target so that it is
  fixed relative to the base of the robot; for example, you can place an AR marker on a table.
- **eye-on-base** to compute the static transform from a robot's base to a tracking system, e.g. the
  optical frame of a camera standing on a tripod next to the robot. In this case you can attach a marker,
  e.g. an AR marker, to the end-effector of the robot.
  

The (arguably) best part is, that you do not have to care about the placement of the auxiliary marker
(the one on the table in the eye-in-hand case, or on the robot in the eye-on-base case). The algorithm
will "erase" that transformation out, and only return the transformation you are interested in.


eye-on-base             |  eye-on-hand
:-------------------------:|:-------------------------:
![](docs/img/eye_on_base_aruco_pic.png)  |  ![](docs/img/eye_on_hand_aruco_pic.png)



It will  be the user's responsibility to make the robot publish its own pose into `tf`. Please check that the robot's pose is updated correctly in 
RViz before starting to acquire samples (the robot driver may not work while the teaching mode button is pressed, etc).

The same applies to the validity of the samples. For the calibration to be found reliably, the end effector must be rotated as much as possible 
(up to 90°) about each axis, in both directions. Translating the end effector is not necessary, but can't hurt either.


## Tips for accuracy

The following tips are given in [1], paragraph 1.3.2.

- Maximize rotation between poses.
- Minimize the distance from the target to the camera of the tracking system.
- Minimize the translation between poses.
- Use redundant poses.
- Calibrate the camera intrinsics if necessary / applicable.
- Calibrate the robot if necessary / applicable.

## References
[1] *Tsai, Roger Y., and Reimar K. Lenz. "A new technique for fully autonomous
and efficient 3D robotics hand/eye calibration." Robotics and Automation, IEEE
Transactions on 5.3 (1989): 345-358.*
