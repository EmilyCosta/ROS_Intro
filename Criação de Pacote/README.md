# Criação de pacotes

Inicialmente é necessário a criação e a iniciação de um workspace.

```Shell
~$ mkdir -p ~/catkin_ws/src
~$ cd ~/catkin_ws/src
~/catkin_ws/src$ catkin_init_workspace
```
A primeira linha de código é a criação de um workspace catkin contendo uma pasta src. No proxima linha adentramos o ws e a pasta e foi iniciado o ws.

```Shell
~/catkin_ws/src$ cd ..
~/catkin_ws$ catkin_make
```

```Shell
~/catkin_ws/src$ catkin_create_pkg <package_name> <dependencies> <language>
```
ex:

```Shell
~/catkin_ws/src$ catkin_create_pkg simple_comunicator std_msgs rospy
```

```Shell
~/catkin_ws/src$ catkin_create_pkg simple_comunicator std_msgs rospy
```

```Shell
~/catkin_ws/src$ cd ~/catkin_ws/src/simple_comunicator
~/catkin_ws/src/simple_comunicator$ git clone <link_scripts> 
```
Caso o Linux não consiga encontrar um package com o nome e o nome não foi digitado de forma errada, é importante lembrar que, antes, deve ser feito ```$ source /devel/setup.zsh``` no terminal em que o programa rodará. 

Para que não seja necessário rodar o comando acima a cada terminal aberto pode-se

```Shell
~$ echo "source /home/SEU_USERNAME/catkin_ws/devel/setup.zsh" >> ~/.zshrc
```
