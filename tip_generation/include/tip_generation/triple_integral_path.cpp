#include "triple_integral_path.h"

// Constructor
TIP::TIP()
{
    j_lim = 0.01;
    a_lim = 0.01;
    v_lim = 0.6;

    hovering_distance = 1.0;

    subscriber_goal = nh.subscribe("/move_base",1,&TIP::goal_msg_callback,this);
    publisher_traj = nh.advertise<traj>("/des_traj",1);

    traj_enable = false;
    is_time_init = false;

}

void TIP::goal_msg_callback(const Pose & goal_msg)
{
    goal_position << goal_msg.position.x,
                    goal_msg.position.y,
                    goal_msg.position.z;

    if(traj_enable == false){
        traj_enable = true;
        time_calculation();
        t0 = ros::Time::now().toSec();
    }
    
}

void TIP::time_calculation()
{
    double t01, t02;
    t01 = j_lim/a_lim;
    t02 = v_lim/a_lim - t01;

    if(goal_position(0) == 0 && goal_position(2) > 0 )
    {
        cout<<"Hovering First"<<endl;
        j_lim = 0.001;
        a_lim = 0.01;
        v_lim = 0.05;

        t01 = sqrt(v_lim/j_lim);
        t1 = t01;
        t2 = t01*2.0;

        z0 = 0;

        a_t1 = j_lim*t1;
        v_t1 = 1/2.0*j_lim*pow(t1,2);
        z_t1 = 1/6.0*j_lim*pow(t1,3);

        v_t2 = -j_lim*t1 + a_t1*t1;
        z_t2 = -j_lim/6.0*pow(t1,3) + a_t1/2.0*pow(t1,2) + v_t1*t1 + z_t1;

        z_stop_start = hovering_distance - z_t2;

        t_stop_start = (hovering_distance - 2*z_t2)/v_lim + t2;
        t3 = t_stop_start + t1;
        t4 = t3 + t1;

        a_t3 = -j_lim*t1;
        v_t3 = -j_lim/2.0*pow(t1,2) + v_lim;
        z_t3 = -j_lim/6.0*pow(t1,3) + v_lim*t1 + z_stop_start;

    /**
        cout<<"t1 : "<<t1<<endl;
        cout<<"t2 : "<<t2<<endl;
        cout<<"t_stop_start : "<<t_stop_start<<endl;
        cout<<"t3 : "<<t3<<endl;
        cout<<"t4 : "<<t4<<endl;
        
        cout<<"Height at t1 : "<<z_t1<<endl;
        cout<<"Height at t2 : "<<z_t2<<endl;
        cout<<"Height at t_stop_start : "<<z_stop_start<<endl;
        cout<<"Height at t3 : "<<z_t3<<endl;
        cout<<"Hovering Distance : "<<z_stop_start+z_t2<<endl;
    **/
    }
    else if(goal_position(0) == 0 && goal_position(2) == 0 )
    {
        cout<<"Landing"<<endl;
        j_lim = -0.001;
        a_lim = -0.01;
        v_lim = -0.05;

        t01 = sqrt(v_lim/j_lim);
        t1 = t01;
        t2 = t01*2.0;

        z0 = hovering_distance;

        a_t1 = j_lim*t1;
        v_t1 = 1/2.0*j_lim*pow(t1,2);
        z_t1 = hovering_distance + 1/6.0*j_lim*pow(t1,3);

        v_t2 = -j_lim*t1 + a_t1*t1;
        z_t2 = -j_lim/6.0*pow(t1,3) + a_t1/2.0*pow(t1,2) + v_t1*t1 + z_t1;

        z_stop_start = hovering_distance - z_t2;

        t_stop_start = (hovering_distance - 2*z_t2)/v_lim + t2;
        t3 = t_stop_start + t1;
        t4 = t3 + t1;

        a_t3 = -j_lim*t1;
        v_t3 = -j_lim/2.0*pow(t1,2) + v_lim;
        z_t3 = -j_lim/6.0*pow(t1,3) + v_lim*t1 + z_stop_start;

    }
    else{
        cout<<"Trajectory Applied"<<endl;
    }

}

void TIP::traj_publish()
{
    t_current = ros::Time::now().toSec() - t0;
    if(traj_enable == true)
        bang_bang();
}

void TIP::bang_bang()
{
    cout<<"Time : "<<t_current<<endl;

    if (t_current >= 0 && t_current < t1)
    {

        traj_data.Accel.linear.x = 0;
        traj_data.Accel.linear.y = 0;
        traj_data.Accel.linear.z = j_lim * t_current;

        traj_data.Twist.linear.x = 0;
        traj_data.Twist.linear.y = 0;
        traj_data.Twist.linear.z = 1/2.0*j_lim*pow(t_current,2);

        traj_data.Pose.position.x = 0;
        traj_data.Pose.position.y = 0;
        traj_data.Pose.position.z = z0+ 1/6.0*j_lim*pow(t_current,3);

    }
    else if(t_current >= t1 && t_current < t2)
    {

        traj_data.Accel.linear.x = 0;
        traj_data.Accel.linear.y = 0;
        traj_data.Accel.linear.z = -j_lim * (t_current - t1) + a_t1;

        traj_data.Twist.linear.x = 0;
        traj_data.Twist.linear.y = 0;
        traj_data.Twist.linear.z = -1/2.0*j_lim*pow((t_current - t1),2)
                    + a_t1*(t_current - t1)
                    + v_t1;

        traj_data.Pose.position.x = 0;
        traj_data.Pose.position.y = 0;
        traj_data.Pose.position.z = -1/6.0*j_lim*pow((t_current - t1),3)
                    + a_t1/2.0*pow((t_current - t1),2)
                    + v_t1*(t_current - t1)
                    + z_t1;
    }
    else if(t_current >= t2 && t_current < t_stop_start)
    {

        traj_data.Accel.linear.x = 0;
        traj_data.Accel.linear.y = 0;
        traj_data.Accel.linear.z = 0;

        traj_data.Twist.linear.x = 0;
        traj_data.Twist.linear.y = 0;
        traj_data.Twist.linear.z = v_lim;

        traj_data.Pose.position.x = 0;
        traj_data.Pose.position.y = 0;
        traj_data.Pose.position.z = z_t2 + v_lim*(t_current - t2);

    }
    else if(t_current >= t_stop_start && t_current < t3)
    {
        traj_data.Accel.linear.x = 0;
        traj_data.Accel.linear.y = 0;
        traj_data.Accel.linear.z = -j_lim*(t_current - t_stop_start);

        traj_data.Twist.linear.x = 0;
        traj_data.Twist.linear.y = 0;
        traj_data.Twist.linear.z = -j_lim/2.0*pow((t_current - t_stop_start),2)
                    + v_lim;

        traj_data.Pose.position.x = 0;
        traj_data.Pose.position.y = 0;
        traj_data.Pose.position.z = -j_lim/6.0*pow((t_current - t_stop_start),3)
                    + v_lim*(t_current - t_stop_start)
                    + z_stop_start;
    }
    else if (t_current >= t3 && t_current < t4)
    {
        traj_data.Accel.linear.x = 0;
        traj_data.Accel.linear.y = 0;
        traj_data.Accel.linear.z = j_lim*(t_current - t3) + a_t3;

        traj_data.Twist.linear.x = 0;
        traj_data.Twist.linear.y = 0;
        traj_data.Twist.linear.z = j_lim/2.0*pow((t_current - t3),2)
                    + a_t3*(t_current - t3)
                    + v_t3;

        traj_data.Pose.position.x = 0;
        traj_data.Pose.position.y = 0;
        traj_data.Pose.position.z = j_lim/6.0*pow((t_current - t3),3)
                    + a_t3/2.0*pow((t_current - t3),2)
                    + v_t3*(t_current - t3)
                    + z_t3;
    }
    else if(t_current >= t4)
    {
        traj_data.Accel.linear.x = 0;
        traj_data.Accel.linear.y = 0;
        traj_data.Accel.linear.z = 0;

        traj_data.Twist.linear.x = 0;
        traj_data.Twist.linear.y = 0;
        traj_data.Twist.linear.z = 0;

        traj_data.Pose.position.x = 0;
        traj_data.Pose.position.y = 0;
        traj_data.Pose.position.z = j_lim/6.0*pow(t1,3)
                    + a_t3/2.0*pow(t1,2)
                    + v_t3*t1
                    + z_t3;

        traj_enable = false;
    }
    publisher_traj.publish(traj_data);
}

TIP::~TIP()
{

}