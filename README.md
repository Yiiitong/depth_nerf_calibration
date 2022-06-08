# depth_nerf_calibration

## Intrinsic calibration
intrinsic calibration for realsense D435
~~~
roslaunch rs_d435.launch
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.03 image:=/rs_d435/color/image_raw camera:=/rs_d435 --no-service-check
~~~
Save the calibrated intrinsic into d435_intrinsic_larger.yaml and publish the updated camera info
~~~
rostopic pub -r 3000 /rs_d435/color/new_camera_info sensor_msgs/CameraInfo -f d435_intrinsic_larger.yaml
~~~

## Extrinsic calibration

### Using easy hand-eye
~~~
roslaunch hand_eye_calibration_ndi_rs.launch
~~~

### Using fixed board as reference

1. Fix board, fix polaris (don't move them afterwards).

3. Use pointer_tip tracker to get coordinates of 12 corners on the charuco board:
~~~
roslaunch get_pointcoordinate.launch
~~~
get save_points_pointer_tip_np.txt

3. use rigid_transform_3D.py to get pose of the board from the 12 coordinates saved in save_points_pointer_tip_np.txt. (get x y z x y z w)(translation + quaterion)
~~~
python3 rigid_transform_3D.py
~~~

4. put the pose result into check_calibration_use_marker.launch:
~~~
    <node pkg="tf" type="static_transform_publisher" name="fix_marker" args="0.12178378 -0.13873586 -1.02622315 -0.09006817  0.04294831  0.70965433  0.69744813 optical_origin marker 100" />
~~~

5. run check_calibration_use_marker.launch
~~~
roslaunch check_calibration_use_marker.launch
~~~
real-time extrinsic result wil be printed
Also the error between ground truth board corner coordinates and reprojected corner coordinates will be printed

Example extrinsic result:
~~~
0.05915086 -0.04666373 -0.07656602  0.70759693  0.69656685 -0.11808417  0.00278902
~~~
For calibration evaluation, put the extrinsic result into check_calibration_use_marker.launch
~~~
     <node pkg="tf" type="static_transform_publisher" name="fix_calibration" args="0.05915086 -0.04666373 -0.07656602  0.70759693  0.69656685 -0.11808417  0.00278902  $(arg robot_effector_frame) $(arg tracking_base_frame) 100" />
~~~
Both rotation error (degree) and translation error (mm) will be printed

