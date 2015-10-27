#include <Arduino.h>

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

Servo servo;

int servo_angle = 0;
int w = 0;
long r=0;
int pos = 3;

void close_eye(const std_msgs::Empty& msg) {
    r= random(500,5000);
    for(w = 0; w < 30; w += pos)  // goes from 0 degrees to 180 degrees
    {
        servo.write(w);              // tell servo to go to position in variable 'pos'
        delay(10);                       // waits 15ms for the servo to reach the position
    }
    for(w = 30; w>=1; w-=pos*3)     // goes from 180 degrees to 0 degrees
    {
        servo.write(w);              // tell servo to go to position in variable 'pos'
        delay(3);                       // waits 15ms for the servo to reach the position
    }
    delay(r);
}

ros::Subscriber<std_msgs::Empty> sub("close_eye", close_eye);
void setup(){
    pinMode(13, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
    servo.attach(9); //attach it to pin 9
}

void loop(){
    nh.spinOnce();
    delay(1);
}
