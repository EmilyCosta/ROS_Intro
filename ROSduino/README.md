# Comunicação ROS com arduino

Instalação package rosserial.
```SHELL
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```
Instalação biblioteca ros_lib no arduino

```SHELL
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```