#ifndef TVC_CONTROL_H
#define TVC_CONTROL_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <lm4075e_msgs/Int32.h>
#include <actuator_msgs/actuator.h>
#include <iostream>

using std::cout;
using std::endl;

using lm4075e_msgs::Int32;
using actuator_msgs::actuator;
using Eigen::Vector3d;
using Eigen::Matrix3d;

const double Length2Count = 1000.0/0.0460; // 1000 count = 0.046 m
const double Length_init = 0.360; // Length at min stroke = 0.243 m

class tvc_test{

    public:

    // Constructor
    tvc_test();
    
    // Desired Roll Pitch Callback Function
    void CallbackDesRollPitch(const actuator & rp_des);
    void InverseKinematics(double &phi, double &theta);
    double signum(double & theta_ptr);

    // Destructor
    ~tvc_test();

    private:
    ros::NodeHandle nh;
    ros::Subscriber roll_pitch_subscriber;
    ros::Publisher des_pos_publisher;

    // Parameter setup
    double x_lower, width_lower, z_lower;
    double x_upper, width_upper, z_upper;

    // Upper Joint w.r.t lower plate
    Vector3d P_a_left;
    Vector3d P_a_right;

    // Lower Joint w.r.t upper plate
    Vector3d P_b_left;
    Vector3d P_b_right;

    // rp_des data
    double phi, theta;
    Matrix3d RotM;

    // Position(Count) data to publish
    int pos_l, pos_r;

    // Publish data
    Int32 pos_data;

    double theta_max;

};


#endif
