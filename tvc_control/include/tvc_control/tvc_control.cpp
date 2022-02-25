#include "tvc_control.h"

tvc_test::tvc_test()
{
    cout<<"\n";
    cout<<"\n";
    
    cout<<"Constructor"<<endl;

    cout<<"Get Parameter from launch xml file"<<endl;

    nh.getParam("x_lower",x_lower);
    nh.getParam("width_lower",width_lower);
    nh.getParam("z_lower",z_lower);

    nh.getParam("x_upper",x_upper);
    nh.getParam("width_upper",width_upper);
    nh.getParam("z_upper",z_upper);

    roll_pitch_subscriber = nh.subscribe("/des_roll_pitch",1,&tvc_test::CallbackDesRollPitch,this);
    des_pos_publisher = nh.advertise<actuator>("/des_pos",1);

    cout<<"*****Joint Info*****"<<endl;

    P_a_left << x_lower, width_lower/2.0, z_lower;
    P_a_right << x_lower, -width_lower/2.0, z_lower;

    P_b_left << x_upper, width_upper/2.0, z_upper;
    P_b_right << x_upper, -width_upper/2.0, z_upper;    
    
    cout<<"--Lower Joint info: "<<endl;
    cout<<"----Lower Joint Left: "<<endl;
    cout<<P_a_left<<endl;
    cout<<"----Lower Joint Right: "<<endl;
    cout<<P_a_right<<endl;
    
    cout<<"--Upper Joint info: "<<endl;
    cout<<"----Upper Joint Left: "<<endl;
    cout<<P_b_left<<endl;
    cout<<"----Upper Joint Right: "<<endl;
    cout<<P_b_right<<endl;

    cout<<"--Initial Link Length: "<<endl;
    cout<<Length_init<<endl;
    

    cout<<"Initialize Rotation matrix"<<endl;
    RotM<<1,0,0,
        0,1,0,
        0,0,1;
    cout<<"--RotM"<<endl;
    cout<<RotM<<endl;

    cout<<"--Pitch Max Setup--"<<endl;
    cout<<"Launch file - degree, convert to radian in the code"<<endl;
    nh.getParam("theta_max",theta_max);
    theta_max = theta_max*M_PI/180.0;

    cout<<"----Pitch Max (rad): "<<theta_max<<endl;

    cout<<"\n";
    cout<<"\n";
    
}


void tvc_test::CallbackDesRollPitch(const actuator & rp_des)
{
    phi = rp_des.phi_u;
    theta = rp_des.theta_u;

    if ( abs(theta) > theta_max)
        theta = theta_max*signum(theta);

    InverseKinematics(phi,theta);

}

void tvc_test::InverseKinematics(double &phi, double &theta)
{
    // Convention 
    RotM << cos(phi), sin(theta)*sin(phi), sin(theta)*cos(phi),
            0, cos(phi), -sin(phi),
            -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi);

    pos_l = (int) Length2Count*sqrt((RotM*P_a_left - P_b_left).dot(RotM*P_a_left - P_b_left));
    pos_l = pos_l - Length_init*Length2Count;

    pos_r = (int) Length2Count*sqrt((RotM*P_a_right - P_b_right).dot(RotM*P_a_right - P_b_right));
    pos_r = pos_r - Length_init*Length2Count;
    
    pos_data.left_pos = pos_l;
    pos_data.right_pos = pos_r;

    des_pos_publisher.publish(pos_data);
}

double tvc_test::signum(double & theta_ptr)
{
    if (theta_ptr > 0.0)
        return 1.0;
    
    if(theta_ptr < 0.0)
        return -1.0;
    return 0.0;
}

tvc_test::~tvc_test()
{
    cout<<"Destructor"<<endl;

    roll_pitch_subscriber.~Subscriber();
    des_pos_publisher.~Publisher();

    cout<<"\n";
    cout<<"\n";
    
}