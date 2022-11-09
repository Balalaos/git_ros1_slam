# robominer_simulator
Simulation of robominer (ROS + Gazebo)

You need to install the following packages:

```
sudo apt-get install ros-melodic-joint-state-controller
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-position-controllers
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```

Also, you need to add the models folder to Gazebo's path: 

```
echo 'export GAZEBO_MODEL_PATH=~/catkin_ws/src/robominer_simulation/models' >> ~/.bashrc 

```


# To launch the simulator:
```
roslaunch robominer_simulation main.launch
```