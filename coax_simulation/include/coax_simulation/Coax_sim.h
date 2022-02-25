#ifndef COAX_SIM_H
#define COAX_SIM_H

#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

#include <iterator>
#include <algorithm>
#include <iostream>

#include <std_msgs/UInt16.h>
#include <tvc_control/rollpitch.h>
#include <actuator_msgs/actuator.h>

#include <nav_msgs/Odometry.h>

using boost::numeric::odeint::runge_kutta4;

using std::cout;
using std::endl;

using std_msgs::UInt16;
using actuator_msgs::actuator;
using tvc_control::rollpitch;

using nav_msgs::Odometry;

typedef Eigen::Matrix<double, 13, 1> state_type;

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;

void setup_global_variables();
void system_dynamics(state_type &s, state_type &dsdt, double t);
void Quat2Rot(Quaterniond &quat);
void Euler2Rot(double &phi,double &theta);

class Coax_sim{
    
    public:

        // Constructor
        Coax_sim();
 
        void cb_actuator(const actuator& actuator);

        void do_RK4();

        void publish_state();

        // Destructor
        ~Coax_sim();

    private:

        ros::NodeHandle nh;

        ros::Publisher state_publisher;
        ros::Subscriber actuator_subscriber;

        double current_time;
        double previous_time;
        bool init_time;
        double t0;
        double dt;

        state_type s;
        // s(0) ~ s(2) : vx, vy, vz
        // s(3) ~ s(5) : x, y, z
        // s(6) ~ s(8) : wx, wy, wz
        // s(9) ~ s(12) : qx, qy, qz, qw
        Odometry state_;

        runge_kutta4<state_type> rk4;

};
#endif