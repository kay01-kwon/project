#include "coax_taxing.h"

CoaxTAX::CoaxTAX()
{
    nh.getParam("neutral_roll",neutral_roll);
    nh.getParam("neutral_pitch",neutral_pitch);

    nh.getParam("droll",droll);
    nh.getParam("dpitch",dpitch);

    nh.getParam("Kp_phi",Kp_phi);
    nh.getParam("Kd_phi",Kd_phi);

    actuator_subscriber = nh.subscribe("/flag",1,&CoaxTAX::CallbackActuation,this);
    odometry_subscriber = nh.subscribe("/mavros/odometry/in",1,&CoaxTAX::CallbackOrientation,this);
    actuator_publisher = nh.advertise<actuator>("/actuation",1);
    
}

void CoaxTAX::CallbackActuation(const Int8 & actuation)
{

    flag = actuation.data;

}

void CoaxTAX::CallbackOrientation(const Odometry & ori)
{
    double qx, qy, qz, qw;
    double wx;
    
    qx = ori.pose.pose.orientation.x;
    qy = ori.pose.pose.orientation.y;
    qz = ori.pose.pose.orientation.z;
    qw = ori.pose.pose.orientation.w;
    
    wx = ori.twist.twist.angular.x;

    roll_state = asin(2*(qy*qz + qw*qx));

    roll_u = -Kp_phi*roll_state - Kd_phi*wx;


    if(flag == 1)
    {
        pitch_u = neutral_pitch + dpitch;
    }
    else if(flag == 0)
    {
        pitch_u = neutral_pitch;
    }
    else if(flag == -1)
    {
        pitch_u = neutral_pitch - dpitch;
    }

    act.phi_u = - pitch_u;
    act.theta_u = - roll_u;
}

void CoaxTAX::PublishActuation()
{
    actuator_publisher.publish(act);
}

CoaxTAX::~CoaxTAX()
{

}