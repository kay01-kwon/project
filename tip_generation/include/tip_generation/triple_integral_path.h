#ifndef TRIPLE_INTEGRAL_PATH_H
#define TRIPLE_INTEGRAL_PATH_H

#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Float64.h>

#include <tip_generation/traj.h>
#include <geometry_msgs/Pose.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using std_msgs::Float64;

using geometry_msgs::Pose;
using tip_generation::traj;

using Eigen::Vector3d;

using std::cout;
using std::endl;

class TIP{
    
    public:
    // Constructor
    TIP();

    void goal_msg_callback(const Pose & goal_msg);

    void time_calculation();

    void bang_off_bang();

    void bang_bang();

    bool goal_check();

    void traj_publish();

    ~TIP();

    private:

    ros::NodeHandle nh;

    ros::Subscriber subscriber_goal;
    
    ros::Publisher publisher_traj;

    double j_lim;
    double a_lim;
    double v_lim;

    double z0;

    double a_t1;
    double v_t1;
    double z_t1;

    double a_t2;
    double v_t2;
    double z_t2;

    double z_stop_start;

    double a_t3;
    double v_t3;
    double z_t3;

    double hovering_distance;

    double t_stop_start;

    double t0;
    double t1;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;

    Vector3d goal_position;

    double t_current;

    bool traj_enable;

    bool is_time_init;

    traj traj_data;

};


#endif