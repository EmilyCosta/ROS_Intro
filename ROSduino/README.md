# Comunicação ROS com arduino

## Instalação package rosserial

```SHELL
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```
## Instalação biblioteca ros_lib no arduino

```SHELL
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

## Habilitando a comunicação

Iniciar o roscore:

```SHELL
$ roscore
```

Iniciar o nó de comunicação:

```SHELL
$ rosrun rosserial_arduino serial_node.py _port:=/dev/ttyUSB0
```
