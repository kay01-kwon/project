#include <Arduino.h>
#include <Servo.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <ros.h>
#include <actuator_msgs/actuator.h>

#define CPU_CLK 16000000UL

const int neutral = 90;
Servo servo_left;
Servo servo_right;

using actuator_msgs::actuator;
ros::NodeHandle nh;

void msgCallback(const actuator& msg)
{
  int l,r;

  l = msg.left_servo + 90;
  r = msg.right_servo + 90;

  servo_left.write(l);
  servo_right.write(r);
  
  OCR2A = 155;
  OCR2B = (uint16_t)(msg.main_rotor_pwm/12.50 + 1000.0/125.0);
}

ros::Subscriber<actuator> sub("actuation",msgCallback);

void setup() {
  // put your setup code here, to run once:
nh.initNode();
nh.subscribe(sub);

TCCR2A = 0x00;
TCCR2A = 0x21;
PORTD = 0x00;
DDRD = 0xFF;
TCCR2B = (1<<CS22) | (0<<CS21) |(1<<CS20)| (1<<WGM22);
OCR2A = 155;
OCR2B = 0;

servo_left.attach(5);
servo_right.attach(6);

servo_left.write(90);
servo_right.write(90);

}

void loop() {
  // put your main code here, to run repeatedly:
nh.spinOnce();
delay(1);
}
