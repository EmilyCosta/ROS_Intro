# Códigos

Aqui ficarão disponíveis os programas principais utilizados no desenvolvimento de aplicações que utilizam o ROS.


## pub.py

```python
#!/usr/bin/env  python3

# Module import
import rospy #specific library to program ROS in Python
from random import randint #generates random values
from std_msgs.msg import String #type of message/variable that will be used

# Global variables
L = []

# Defining functions
def listToString(s): 
    '''Transforms a list into String'''
    str1 = " "      #crates an empty String
    return (str1.join(s))      #returns String

def Publisher(data):
    '''ROS adaptations'''
    pub = rospy.Publisher('Data_COM', String, queue_size = 10) #Data_COM is the name of the topic
    rospy.init_node('Test_pub_COM', anonymous = True)  #defines the node name as 'Test_pub_COM'
    rate = rospy.Rate(10)    #loop frequency - x loops/s (Hz)

    #while not rospy.is_shutdown():  #keeps sendind data until stopped (without it, sends one set of data and then terminates)
    rospy.loginfo(data)
    pub.publish(data)
    rate.sleep()

# Main program
if __name__ == '__main__':
    
    try:
        for i in range (10): #the range defines the amount of data that will be sent at once
            L.append(str(randint(0,100))) 
        L.append('-1') #the '-1' is used as a stop character
        Publisher(listToString(L))
    except rospy.ROSInterruptException:  #to prevent the code to run inside rate.sleep()
        pass
```

## blink.ino

```cpp
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

#define LED_PIN 13

void messageCb( const std_msgs::UInt16 &toggle_led );

std_msgs::String str_msg;
ros::Publisher pub("H_L", &str_msg);
ros::Subscriber<std_msgs::UInt16> sub("toggle_led", &messageCb);

std_msgs::UInt16 led_state;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub);
}

void loop() {
    nh.spinOnce();
    delay(1);
}

void messageCb( const std_msgs::UInt16 &toggle_led) {
    led_state = toggle_led.data;
    digitalWrite(LED_PIN, led_state);
    sprintf(str_msg.data, "%s", led_state ? "H" : "L");
    pub.publish( &str_msg );
}
```

## rand_b.py

```python
#!/usr/bin/env python3

# Module import
import rospy
from std_msgs.msg import UInt16
from random import randint

# Defining functions
def pub_info(data):
    ''''Publishes the desired data on the 'toggle_led' topic'''
    pub = rospy.Publisher('toggle_led', UInt16, queue_size=1) 
    pub.publish(data)


# Main Program
if __name__ == '__main__':
    try:
        rospy.init_node('rand_node', anonymous=True)  
        while not rospy.is_shutdown():
            n = randint(1,512)

            if n % 2 == 0:
                info = 1
            else:
                info = 0

            pub_info(info)     

    except rospy.ROSInterruptException:
        pass
```


## teleop_twist_keyboard.py

```python
#!/usr/bin/env python3

from __future__ import print_function

import threading

from math import sin,cos,radians,pi #math functions used in the kinetic implementation


import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

anything else : stop

8/2 : increase/decrease speeds by 0.5

CTRL-C to quit
""" #adjustment of the msg (new keys utilized)

moveBindings = {
#omnidirectional Movement - sends velocity vector (polar)
    'q':(1,3*pi/4),
    'w':(1,pi/2),
    'e':(1,pi/4),
    'a':(1,pi),
    's':(0,0),
    'd':(1,0),
    'z':(1,5*pi/4),
    'x':(1,3*pi/2),
    'c':(1,7*pi/4),
    '':(0,0),   
    }

speedBindings={
        '8':(.5,1),
        '2':(-.5,1),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0 #z coordinate was removed, because it wouldn't be used
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        #new_message doesn't wait for new data to publish
        self.timeout = 0.01

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y,th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0,0,0,0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = 0 #z coordinate won't be used
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.1) #we set the timeout to 0.1
    if key_timeout == 0.0:
        key_timeout = 0.1

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]*sin((moveBindings[key][1])) #polar to cartesian
                y = moveBindings[key][0]*cos((moveBindings[key][1]))
                th = moveBindings[key][1]
            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                if speed > 5:
                    speed = speed + speedBindings['2'][0]

                elif speed < 0:
                    speed = speed + speedBindings['8'][0]
                
                

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            
            else:
                # Skip updating cmd_vel if key timeout and robot already stopped.
                if key == '' and x == 0 and y == 0 and th == 0:  #z will always be 0
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
```

## pub_vel_tele.py

```python
#!/usr/bin/env python3

# Module import
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from math import sin,cos,pi,sqrt  #math functions utilized


# Global variables
key_vel_lin = Float32MultiArray()
alpha1 = 0
alpha2 = alpha1+2*pi/3
alpha3 = alpha2+2*pi/3


# Defining functions
def callback(data):
    '''Reads the linear velocity data'''
    vel = data.linear
    turn = 0
    velocity(key_vel_lin,vel,turn)

    
def Read_Keys_Vel():  
    '''Subscribes to the 'cmd_vel' topic'''
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

def velocity(key_vel_lin, vel,turn):
    '''Transforms the Twist message into the desired Float32MultiArray'''
    key_vel_lin.data.clear()



    v1 = -sin(turn+alpha1)*vel.x+cos(turn+alpha1)*vel.y #+sqrt(vel.x**2+vel.y**2)
    v2 = -sin(turn+alpha2)*vel.x+cos(turn+alpha2)*vel.y #+sqrt(vel.x**2+vel.y**2)
    v3 = -sin(turn+alpha3)*vel.x+cos(turn+alpha3)*vel.y #+sqrt(vel.x**2+vel.y**2)
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
```


## Controle_ROS

```cpp
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <Encoder.h>

/****************ROS Definitions****************/

//Subcriber
void velCb( const std_msgs::Float32MultiArray &set_msg ); // callback
ros::Subscriber<std_msgs::Float32MultiArray> sub("set", &velCb);

//Publisher
std_msgs::Float32MultiArray vel_msg; // variavel pub
ros::Publisher pub("vel", &vel_msg);

//Serial Port Definition
class NewHardware : public ArduinoHardware
{
  public:
    NewHardware(): ArduinoHardware(&Serial3, 57600) {};
};
ros::NodeHandle_<NewHardware>  nh;


/****************Engines Definitions****************/

Encoder ENC1(2, 3); //Engine 1
Encoder ENC2(18, 19); //Engine 2
Encoder ENC3(20, 21); //Engine 3


//Engine 1 Variables
float vel1 = 0;
int out1 = 0 ;
int out1_ant = 0 ;
float error1 = 0;
float error1_ant = 0;
float a_1 = 151.2;
float b_1 = 138.1;
long Enc1_ant = 0;
long Enc1 = 0;

//Engine 2 Variables
float vel2 = 0;
int out2 = 0 ;
int out2_ant = 0 ;
float error2 = 0;
float error2_ant = 0;
float a_2 = 130;
float b_2 = 118.4;
long Enc2_ant = 0;
long Enc2 = 0;

//Engine 3 Variables
float vel3 = 0;
int out3 = 0 ;
int out3_ant = 0 ;
float error3 = 0;
float error3_ant = 0;
float a_3 = 144.9;
float b_3 = 132.1;
long Enc3_ant = 0;
long Enc3 = 0;


float set [] = {0, 0, 0};

#define PWM1 10
#define PWM2 7
#define PWM3 5
#define DIR1 9
#define DIR2 6
#define DIR3 4

unsigned long T0 = 0;
unsigned long T1 = 0;

void setup() {

  TCCR2B = TCCR2B & B11111000 | B00000010; //Timer 2
  TCCR3B = TCCR3B & B11111000 | B00000010; //Timer 3
  TCCR4B = TCCR4B & B11111000 | B00000010; //Timer 4


  Serial3.begin(57600);
  Serial3.write("AT&F,ATS1=57,ATS2=64,ATS5=0,AT&W,ATZ"); //RTK module configuration

  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  if (millis() - T0 >= 10) {
    Enc1 = ENC1.read();
    vel1 = (Enc1 - Enc1_ant) / (5280 * 0.01);
    Enc1_ant = Enc1;
    error1 = set[0] - vel1;
    out1 = out1_ant + int(a_1 * error1 - b_1 * error1_ant);
    if (out1 > 255)out1 = 255;
    if (out1 < -255)out1 = -255;
    out1_ant = out1;
    error1_ant = erro1;

    Enc2 = ENC2.read();
    vel2 = (Enc2 - Enc2_ant) / (5280 * 0.01);
    Enc2_ant = Enc2;
    error2 = set[1] - vel2;
    out2 = out2_ant + int(a_2 * error2 - b_2 * error2_ant);
    if (out2 > 255)out2 = 255;
    if (out2 < -255)out2 = -255;
    out2_ant = out2;
    erro2_ant = erro2;

    Enc3 = ENC3.read();
    vel3 = (Enc3 - Enc3_ant) / (5280 * 0.01);
    Enc3_ant = Enc3;
    error3 = set[2] - vel3;
    out3 = out3_ant + int(a_3 * error3 - b_3 * error3_ant);
    if (out3 > 255)out3 = 255;
    if (out3 < -255)out3 = -255;
    out3_ant = out3;
    error3_ant = error3;
    T0 = millis();
  }

  engine(out1, PWM1, DIR1);
  engine(out2, PWM2, DIR2);
  engine(out3, PWM3, DIR3);

}


void engine(int vel, int pinvel, int pindir) {
  if (vel < 0) {
    digitalWrite(pindir, LOW);
  }
  else {
    digitalWrite(pindir, HIGH);
  }
  analogWrite(pinvel, abs(vel));
}

void velCb( const std_msgs::Float32MultiArray &set_msg) {
  for (int i = 0; i < 3; i++) {
    set[i] = float(set_msg.data[i]);
  }
}
```
