#ifndef COAX_TAXING_H
#define COAX_TAXING_H

#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Int8.h>
#include <actuator_msgs/actuator.h>
#include <nav_msgs/Odometry.h>


using std_msgs::Int8;
using actuator_msgs::actuator;
using nav_msgs::Odometry;

using std::cout;
using std::endl;

class CoaxTAX{

    public:

    //Constructor
    CoaxTAX();

    //Callback Int8
    void CallbackActuation(const Int8 & actuation);

    //Callback Ori
    void CallbackOrientation(const Odometry & ori);

    void PublishActuation();

    ~CoaxTAX();

    private:

    ros::NodeHandle nh;
    ros::Subscriber actuator_subscriber;
    ros::Subscriber odometry_subscriber;
    ros::Publisher actuator_publisher;

    actuator act;

    double roll_u;
    double pitch_u;

    double neutral_roll;
    double neutral_pitch;

    double roll_state;

    double droll;
    double dpitch;

    double Kp_phi;
    double Kd_phi;

    int flag;

};

#endif