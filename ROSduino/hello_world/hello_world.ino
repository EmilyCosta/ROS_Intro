#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher pub("message", &str_msg);

char hello[13] = "Hello ROS";

void setup() {
  nh.initNode();
  nh.advertise(pdp);
}

void loop() {
  str_msg.data = hello;
  pdp.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
