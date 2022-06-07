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

4. put the pose result into check_calibration.launch:
~~~
   <node pkg="tf" type="static_transform_publisher" name="fix_marker" args="0.04417303 -0.13011689 -0.96095168  -0.11778477  0.15470386  0.70250718  0.68459997 optical_origin board 100" />
~~~

5. run check_calibration_use_marker.launch
~~~
roslaunch check_calibration_use_marker.launch
~~~
