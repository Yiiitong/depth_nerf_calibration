# RoboticVisionDataset

## Setup

### Install RealSense
[Instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)

### Install ArenaSDK
[Instructions](https://thinklucid.com/downloads-hub/)
```
export ARENA_ROOT=/opt/ArenaSDK_Linux_x64
export ARENA_CONFIG_ROOT=~/catkin_ws
```

Documentation under
```
/opt/ArenaSDK_Linux_x64/docs/html/arena_sdk_linux.html
```

### Install ROS
[Instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)

### Update gazebo version
```
sudo apt-get remove *gazebo*
sudo apt-get install ros-melodic-gazebo11-ros-control ros-melodic-gazebo11-ros ros-melodic-gazebo11-msgs ros-melodic-gazebo11-plugins ros-melodic-gazebo11-ros-pkgs ros-melodic-gazebo11-dev ros-melodic-ddynamic-reconfigure
```

## IIWA Robot Setup
### Internal resources
[ROS Setup](https://dc.campar.in.tum.de/t/starting-with-ros-and-imfusion/479)

[Launchfile](https://dc.campar.in.tum.de/t/roboticultrasound-calibration/415)
### Setup Ethernet

#### Setup device
##### Temporary assignment
Set IP, netmask and mtu
```
sudo ifconfig <device_name> 172.31.1.150 netmask 255.255.255.0
```

##### Alternative
###### Create config file
```
# Let NetworkManager manage all devices on this system
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    <device_name>:
      dhcp4: no
      addresses:
        - 172.31.1.150/24
```
###### Copy and activate
(Maybe backup the old config file first)
```
sudo cp 01-network-manager-all.yaml /etc/netplan/01-network-manager-all.yaml
sudo netplan apply
```

#### Set ROS Ip and URI
```
export ROS_IP=172.31.1.150
export ROS_MASTER_URI=http://$ROS_IP:11311
```

#### Ping robots to test connection
```
# Robot close to the pillar
ping 172.31.1.145 -c 3
# Robot closer to the wall
ping 172.31.1.147 -c 3
```

### Setup iiwa stack
From inside ~/catkin_ws
1. Source ros `source /opt/ros/melodic/setup.bash`
2. Clone repository `git clone https://github.com/IFL-CAMP/iiwa_stack.git src/iiwa_stack`
3. Build workspace `catkin build`
4. Source workspace `source devel/setup.bash`

### Run visualization
Make sure Robot is in `AUT` mode in top left corner of the window of the control panel, if it is in `T1` or `T2` rotate key at top center on control clock wise and change the mode.

1. Run roscore and rviz
```
roslaunch iiwa_moveit moveit_planning_execution.launch sim:=false
```
2. Start SmartServo application on robot control panel, see under applications on the top and then press play button on control panel to start it.
3. Once finished press stop button on robot control first, then stop roscore.

## Lucid Cameras
### Copy image_encoding.h
```
sudo cp ~/catkin_ws/src/arena_camera_ros/image_encodings.h /opt/ros/melodic/include/sensor_msgs/image_encodings.h
```
### Assign names to devices
Connect devices one by one and assign a name
```
rosrun arena_camera write_device_user_id_to_camera <name>
```

### Setup Ethernet
#### Adjust Ubuntu memory settings
Append the following to `/etc/sysctl.conf`
```
# Custom
fs.inotify.max_user_watches=524288
net.core.rmem_default=16777216
net.core.rmem_max=16777216
net.ipv4.conf.default.rp_filter=0
net.ipv4.conf.all.rp_filter=0
```
#### Setup devices
Assign each device to its own subnet.
Set IP, netmask and mtu using the following commands.
```
sudo ifconfig <device_name> X.X.X.X netmask 255.255.0.0 mtu 9000
sudo ethtool -G <device_name> rx 4096
```

Assign static IP to each device using the IpConfigUtility tool in Arena SDK located under
```
/opt/ArenaSDK_Linux_x64/precompiledExamples
```

List devices
```
./IpConfigUtility /list
```

Set IP first
```
./IpConfigUtility /force -m <MAC> -a <IP> -s <subnetmask> -g 0.0.0.0
```

Set IP permanently
```
./IpConfigUtility /persist -m <MAC> -p true -a <IP> -s <subnetmask> -g 0.0.0.0
```

#### Example configuration
```
./IpConfigUtility /list
[index]	MAC             IP              SUBNET          GATEWAY         	IP CONFIG
[0]	1C0FAF1793C4    192.170.3.30    255.255.0.0     0.0.0.0         	DHCP= 0 Persistent Ip= 1 LLA = 1

[1]	1C0FAF0DCACA    192.169.2.20    255.255.0.0     0.0.0.0         	DHCP= 0 Persistent Ip= 1 LLA = 1

[2]	1C0FAF0DE461    192.168.1.10    255.255.0.0     0.0.0.0         	DHCP= 0 Persistent Ip= 1 LLA = 1
```

```
ifconfig
enx00e04c682985: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 9000
        inet 192.169.2.2  netmask 255.255.0.0  broadcast 192.169.255.255
        inet6 fe80::5fa2:18f8:a584:4e4f  prefixlen 64  scopeid 0x20<link>
        ether 00:e0:4c:68:29:85  txqueuelen 1000  (Ethernet)
        RX packets 7  bytes 560 (560.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 49  bytes 7057 (7.0 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

enx00e04c683a8c: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 9000
        inet 192.168.1.1  netmask 255.255.0.0  broadcast 192.168.255.255
        inet6 fe80::7e3e:ff7f:39d3:d7af  prefixlen 64  scopeid 0x20<link>
        ether 00:e0:4c:68:3a:8c  txqueuelen 1000  (Ethernet)
        RX packets 7  bytes 560 (560.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 41  bytes 6360 (6.3 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

enx00e04c683b34: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 9000
        inet 192.170.3.3  netmask 255.255.0.0  broadcast 192.170.255.255
        inet6 fe80::2f32:1409:6713:e8ee  prefixlen 64  scopeid 0x20<link>
        ether 00:e0:4c:68:3b:34  txqueuelen 1000  (Ethernet)
        RX packets 287  bytes 109856 (109.8 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 365  bytes 27324 (27.3 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

```
route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         10.23.0.1       0.0.0.0         UG    101    0        0 eno1
10.23.0.0       0.0.0.0         255.255.255.0   U     101    0        0 eno1
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 enx00e04c682985
192.168.0.0     0.0.0.0         255.255.0.0     U     106    0        0 enx00e04c683a8c
192.169.0.0     0.0.0.0         255.255.0.0     U     105    0        0 enx00e04c682985
192.170.0.0     0.0.0.0         255.255.0.0     U     104    0        0 enx00e04c683b34

```
## Intrinsic calibration
### resources
[ROS Monocular calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)

[Lucid camera calibration](https://support.thinklucid.com/using-ros-for-linux/)
### Stereo camera (left one)
Intrinsic calibration:
```
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.03 image:=/stereo_left/image_raw camera:=/stereo_left
```
After finish sampling, click _Calibrate_ (_Commit_ button will not work for this camera, we should save the result in camera info manually as below)
```
mkdir ~/calibration
tar -xvf /tmp/calibrationdata.tar.gz -C ~/calibration
rosparam set /stereo_left/camera_info_url "file:///home/yitong/calibration/ost.yaml"
```
Run the camera:
```
cd catkin_ws
roslaunch launch/left.launch 
```

## Hand-eye calibration
### Internal resources
[RealSense - robot calibration](https://dc.campar.in.tum.de/t/realsense-robot-calibration/464)

[IFL easy_handeye](https://github.com/IFL-CAMP/easy_handeye)

For stereo camera:

```
cd catkin_ws
roslaunch iiwa_aruco_handeyecalibration stereo_calibrate.launch
```
if run aruco detection separately:

```
cd catkin_ws
roslaunch launch/left.launch 
rviz rviz
roslaunch aruco_ros single.launch marker_id:=556 marker_size:=0.15

```
1. In rviz add “Image” and select the aruco_ros topic (/aruco_single/result) topic from the drop-down menu, as well as add “Marker” and select the aruco_ros topic (/aruco_single/marker). 
2. In case there is an error with the “Global status”, switch the fixed frame (or the map) to camera_color_frame (camera_link)
3. Remember to remap the stereo camera node in launch file:
```
<remap from="/camera_info" to="/stereo_left/camera_info"/>
<remap from="/image" to="/stereo_left/image_rect"/>
```

