#!/usr/bin/env python3

# Module import
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from math import sin,cos,pi,sqrt


# Global variables
key_vel_lin = Float32MultiArray()
alpha1 = 0
alpha2 = alpha1+2*pi/3
alpha3 = alpha2+2*pi/3


# Defining functions
def callback(data):
    '''Reads the linear velocity data'''
    vel = data.linear
    omega = data.angular.z
    turn = 0
    velocity(key_vel_lin,vel,turn,omega)

    
def Read_Keys_Vel():  
    '''Subscribes to the 'cmd_vel' topic'''
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

def velocity(key_vel_lin, vel,turn,omega):
    '''Transforms the Twist message into the desired Float32MultiArray'''
    key_vel_lin.data.clear()



    v1 = -sin(turn+alpha1)*vel.x+cos(turn+alpha1)*vel.y+omega
    v2 = -sin(turn+alpha2)*vel.x+cos(turn+alpha2)*vel.y+omega
    v3 = -sin(turn+alpha3)*vel.x+cos(turn+alpha3)*vel.y+omega
    key_vel_lin.data.insert(0, v1)
    key_vel_lin.data.insert(1, v2)
    key_vel_lin.data.insert(2, v3) 
    
    pub_vel(key_vel_lin)



def pub_vel(data):
    ''''Publishes the desired data on the 'set' topic'''
    pub = rospy.Publisher('set', Float32MultiArray, queue_size=1) 
    pub.publish(data)



# Main Program
if __name__ == '__main__':
    try:
        rospy.init_node('Vel_node', anonymous=True)  
        while not rospy.is_shutdown():
            for list in rospy.get_published_topics():
                if '/cmd_vel' in list:
                    Read_Keys_Vel()

    except rospy.ROSInterruptException:
        pass
