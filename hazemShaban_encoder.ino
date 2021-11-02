#include <ros.h>
#include <std_msgs/Bool.h>
ros::NodeHandle nh;
std_msgs::long long count_msg;
ros::Publisher Encoder_counter("count", &count_msg);

#define sig_A     PB0
#define sig_B    PB1

long long counter = 0;

void setup() {
  nh.initNode();
  nh.advertise(Encoder_counter);
  pinMode(sig_A, INPUT_PULLUP);
  pinMode(sig_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sig_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(sig_B), ISR_B, CHANGE);
}

void loop() {

    count_msg.data =counter ;
    Encoder_counter.publish(&count_msg);

  nh.spinOnce();
}
void ISR_A()
{
  if (digitalRead(sig_A) != digitalRead(sig_B)) {
    counter++;
  }
  else
    counter--;
}
void ISR_B()
{
  if (digitalRead(sig_A) == digitalRead(sig_B)) {
    counter++;
  }
  else
    counter--;
}
