
//#include <ros.h>
//#include <std_msgs/THE_TYPE_OF_THE_MESSAGE_YOU_SUBSCRIBER>
//ros::NodeHandle nh;

//////////////////////////////////////////////////////////
#include <SimplyAtomic.h>
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 6
#define IN2 7
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
float vFilt = 0;
float vPrev = 0;
float vt =0;
float eintegral = 0;
////////////////////////////////////////////////////////////
//void messageCb(const std_msgs::MESSAGE_TYPE& msg)
//{
  //vt= msg.data;
 //}

//ros::Subscriber<std_msgs::MESSAGE_TYPE> sub("THE_TOPIC_THAT_SUBSCRIBER", &messageCb);

///////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder, RISING);
  //nh.initNode();
  //nh.subscribe(sub);
}

void loop() {
   //nh.spinOnce();
  int pos = 0;
  ATOMIC(){
  pos = pos_i;
  }
  //compute the velocity
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float velocity = (pos - posPrev) / deltaT;
  posPrev = pos; 
  prevT = currT;
  
  // Convert count/s to RPM
  float v = velocity / 600.0 * 60.0;

  // Low-pass filter (25 Hz cutoff)
  vFilt = 0.854 * vFilt + 0.0728 * v + 0.0728 * vPrev;
  vPrev = v;
  // Set a target
  //float vt = 100 * (sin(currT / 1e6) > 0);

  // Compute the control signal u
  float kp = 5;
  float ki = 10;
  float e = vt - vFilt;
  eintegral = eintegral + e * deltaT;
  float u = kp * e + ki * eintegral;

  
  // Set the motor speed and direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  
  int pwr = (int) fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }
  
  setMotor(dir, pwr, PWM, IN1, IN2);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(vFilt);
  Serial.println();
  delay(1);
}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Motor speed
  if (dir == 1) {
    // Turn one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0) {
    // If B is high, increment forward
    increment = 1;
  }
  else {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

}
