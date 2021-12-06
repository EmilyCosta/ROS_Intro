#!/usr/bin/env python3

# Module import
from serial import Serial
import rospy
from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from math import sin,cos,pi,sqrt


# Global variables
vel = Float32MultiArray()
alpha1 = 0
alpha2 = alpha1+2*pi/3
alpha3 = alpha2+2*pi/3


# Defining functions

def map(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

def callback(data):
    '''Reads the linear velocity data'''
    velo = 1 #map(data.axes[3],-1,1,0,5)
    x = data.axes[1] *velo
    y = data.axes[0]*velo
    if (data.buttons[8]):
        omega = -velo
    elif (data.buttons[9]):
        omega=velo
    else:
        omega=0

    velocity(vel,x,y,0,omega)
    
    
    
def readJoy():  
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()

def velocity(vel, velx,vely,turn,omega):
    '''Transforms the Twist message into the desired Float32MultiArray'''
    vel.data.clear()


    v1 = -sin(turn+alpha1)*velx+cos(turn+alpha1)*vely+omega
    v2 = -sin(turn+alpha2)*velx+cos(turn+alpha2)*vely+omega
    v3 = -sin(turn+alpha3)*velx+cos(turn+alpha3)*vely+omega
    vel.data.insert(0, v2)
    vel.data.insert(1, v1)
    vel.data.insert(2, v3) 
    
    pub_vel(vel)



def pub_vel(data):
    ''''Publishes the desired data on the 'set' topic'''
    pub = rospy.Publisher('set', Float32MultiArray, queue_size=1) 
    pub.publish(data)



# Main Program
if __name__ == '__main__':
    try:
        rospy.init_node('Vel_node', anonymous=True)  
        while not rospy.is_shutdown():
            readJoy()
            
            
    except rospy.ROSInterruptException:
        pass
