# Turtlebot Navigation

## Components
1. Computer: 
- SLAM + navigation + YOLOv5 + human position estimation
- Ubuntu 20 + ros noetic (ros master) + python 3.8
- hotspot
2. Jetson: 
- wifi adaptor & camera (publish image)
- Ubuntu 18 + ros melodic + python 2.7
- connect to computer hotspot
3. Raspberry Pi: 
- motors & lidar interface (publish sensory feedback & control motor)
- Raspbian + ros kinetic
- connect to computer hotspot

## Running the demo
1. set up the computer as the ros master and share a hotspot. 
- use the following command to check the ip.
```
ifconfig
```
- add the following commands to ```~/.bashrc``` to setup the ros master.
```
export ROS_MASTER_URI=http://{computer ip}:11311
export ROS_IP={computer ip}
```
- source ```~/.bashrc```.
- share the hotspot (using the following command)
```
sudo gnome-control-center
```
- run ```roscore```
2. connect Jetson and Raspberry Pi to screens and the hotspot
3. set up ros ip of Jetson and Raspberry Pi, using the following commands and source ```~/.bashrc```.
```
export ROS_MASTER_URI=http://{computer ip}:11311
export ROS_IP={jetson/rpi ip}
```

### On the Jetson
4. connect the camera and run the following commands to publish the image/video.
```
cd ~/workspace/imgpub
python main.py
```
- after the image/video is shown, you can disconnect the screen

### On the Raspberry Pi
5. run ```roslaunch turtlebot3_bringup turtlebot3_robot.launch``` to start the robot interface.
- after that you can disconnect the screen

### On the Computer
6. follow [this tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node) or [this tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/) to create a map/SLAM

7. follow [this tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation) and [this tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) to perform navigation

