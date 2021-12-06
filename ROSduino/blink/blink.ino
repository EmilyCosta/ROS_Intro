#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

void messageCb( const std_msgs::Empty &toggle_msg );

std_msgs::String str_msg;
ros::Publisher pub("LEDstate", &str_msg);
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

uint8_t led_state = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}

void messageCb( const std_msgs::Empty &toggle_msg) {
  led_state ^= 1;
  digitalWrite(LED_BUILTIN, led_state);
  sprintf(str_msg.data, "%s", led_state ? "H" : "L");
  pub.publish( &str_msg );
}
