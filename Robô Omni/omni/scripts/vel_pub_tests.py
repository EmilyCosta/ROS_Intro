#!/usr/bin/env python3

# Importação de Módulos
from re import M
import threading
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time
from threading import Condition



# Variáveis Globais
Vel_lin = Float32MultiArray()
vel = None

# Definição de Funções

def callback(data):
    vel = data.linear



def readVel(condition): 
    rospy.init_node('Vel_node', anonymous=True) 
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

def velocidade_teclado(Vel_lin, vel, condition):
    Vel_lin.data.clear()

    Vel_lin.data.insert(0, vel.x)
    Vel_lin.data.insert(1, 0)
    Vel_lin.data.insert(2, -vel.x) #Movimento frente/trás: [x  0  x]^T

    pubVel(Vel_lin)

def velocidade_parado(Vel_lin, condition):
    Vel_lin.data.clear()
    condition.acquire()
    while True:
        try:
            callback()

        except:
            val = condition.wait(0.01)
            if val:
                for i in range(3):
                    Vel_lin.data.insert(i, 0)
                pubVel(Vel_lin)

def pubVel(data):
    ''''Publica os dados desjados em algum tópico (ROS)'''
    t = time.time()
    pub = rospy.Publisher('set', Float32MultiArray, queue_size=10)
    ##rospy.init_node('Vel_node', anonymous=True)
    

  
    pub.publish(data)



# Programa Principal
if __name__ == '__main__':
    condition = threading.Condition()
       
    Vel_key = threading.Thread(target=readVel, args=(condition,))
    Vel_key.start()

    Vel_none = threading.Thread(target=velocidade_parado, args=(Vel_lin, condition,))
    Vel_none.start()
    try:
        while not rospy.is_shutdown():
            for lista in rospy.get_published_topics():
                if '/cmd_vel' in lista:
                    readVel()
                else:
                   velocidade_parado(Vel_lin)

    except rospy.ROSInterruptException:
        pass
