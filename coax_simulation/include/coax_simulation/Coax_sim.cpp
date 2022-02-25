#include "Coax_sim.h"

// Dynamic Parameters
double m;               // Mass
Matrix3d I;             // Moment of Inertia
Vector3d CM_p_CM_T;     // COM misalignment

// Gravity
const double g = -9.81;

// Control Input
double main_rotor_pwm;
double T;               // Thrust
Vector3d Thrust_vector;
double C;               // Lift Coefficient
double phi, theta;      // TVC control Input


Matrix3d Euler2Rotm;
Matrix3d Quat2Rotm;

Matrix3d q_v_skiew;
Matrix3d q_xyz;

Matrix3d eye;

Vector3d quat_vec;

// Temporary Rotational Motion State Variables
Vector3d state_w_temp;
Quaterniond state_q_temp;

void setup_global_variables()
{
    m = 40;
    I.setIdentity();
    CM_p_CM_T.setIdentity();
    main_rotor_pwm = 0;
    T = -40.0*g;
    Thrust_vector << 0,0,1;
    C = 0;
    phi = 0;
    theta = 0;

    Euler2Rotm.setIdentity();
    Quat2Rotm.setIdentity();

    q_v_skiew.setIdentity();
    q_xyz.setIdentity();

    eye.setIdentity();
    quat_vec.setZero();

    state_w_temp.setZero();
    quat_vec.setZero();
}

void system_dynamcis(state_type &s, state_type &dsdt, double t)
{

    state_q_temp.x() = s(9);
    state_q_temp.y() = s(10);
    state_q_temp.z() = s(11);
    state_q_temp.w() = s(12);

    quat_vec << state_q_temp.x(),
                state_q_temp.y(),
                state_q_temp.z();

    state_w_temp = s.segment(6,3);

    Quat2Rot(state_q_temp);
    Euler2Rot(phi,theta);

    // 1. Translational dynamics
    dsdt.segment(0,3) = T/m*Quat2Rotm*Euler2Rotm*Thrust_vector;
    dsdt(2) = dsdt(2) + g;

    // 2. Translational kinematics
    dsdt.segment(3,3) = s.segment(0,3);

    // 3. Rotational dynamics
    dsdt.segment(6,3) = I.inverse()*(CM_p_CM_T.cross(Euler2Rotm*T*Thrust_vector)-state_w_temp.cross(I*state_w_temp));

    // 4. Rotational kinematics
    dsdt.segment(9,3) = 0.5*(quat_vec.cross(state_w_temp) + state_q_temp.w()*state_w_temp);
    dsdt(12) = -quat_vec.transpose()*state_w_temp;

    if(s(5) < 0 && T < m*abs(g))
    {    
        dsdt.setZero();
    }
}

void Quat2Rot(Quaterniond &quat)
{
    quat_vec << quat.x(), quat.y(), quat.z();

    q_v_skiew << 0, -quat.z(), quat.y(),
                quat.z(), 0, -quat.x(),
                -quat.y(), quat.x(), 0;
    
    q_xyz = quat_vec*quat_vec.transpose();

    Quat2Rotm = quat.w()*quat.w()*eye + 2*quat.w()*q_v_skiew + 2*q_xyz - quat_vec.transpose()*quat_vec*eye;
}

void Euler2Rot(double &phi,double &theta)
{
    Euler2Rotm << cos(theta), sin(theta)*sin(phi), sin(theta)*cos(phi),
                0, cos(phi), -sin(phi),
                -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi);
}

Coax_sim::Coax_sim()
{
    // Constructor
    s.setZero();
    s(12) = 1;

    cout<<"******** Mass Parameter Setup ********"<<endl;
    nh.getParam("mass",m);
    cout<<"Mass (kg) : ";
    cout<<m<<endl;

    cout<<"******** COM Offset Parameter Setup ********"<<endl;
    nh.getParam("CM_x_CM_T",CM_p_CM_T(0));
    nh.getParam("CM_y_CM_T",CM_p_CM_T(1));
    nh.getParam("CM_z_CM_T",CM_p_CM_T(2));
    cout<<"COM offset : ";
    cout<<CM_p_CM_T<<endl;

    cout<<"******** MOI Parameter Setup ********"<<endl;
    nh.getParam("Ixx",I(0,0));
    nh.getParam("Ixy",I(0,1));
    I(1,0) = I(0,1);
    nh.getParam("Ixz",I(0,2));
    I(2,0) = I(0,2);
    nh.getParam("Iyy",I(1,1));
    nh.getParam("Iyz",I(1,2));
    I(2,1) = I(1,2);
    nh.getParam("Izz",I(2,2));
    cout<<"Moment of Inertia : \n";
    cout<<I<<endl;
    
    cout<<"******** Lift Coefficient Setup ********"<<endl;
    nh.getParam("Lift_Coefficient",C);
    cout<<"Lift Coefficient : ";
    cout<<C<<endl;

    Euler2Rotm.setIdentity();
    Quat2Rotm.setIdentity();

    q_v_skiew.setIdentity();
    q_xyz.setIdentity();

    eye.setIdentity();

    quat_vec.setZero();

    state_w_temp.setZero();
    state_q_temp.setIdentity();


    init_time = false;

    state_publisher = nh.advertise<Odometry>("/mavros/global/local",1);
    actuator_subscriber = nh.subscribe("/actuator",1,&Coax_sim::cb_actuator,this);

}

void Coax_sim::cb_actuator(const actuator& actuator)
{

    //T = C*pow(actuator.main_rotor_pwm,2);
    T = (double)actuator.main_rotor_pwm;
    phi = actuator.phi_u;
    theta = actuator.theta_u;


}

void Coax_sim::do_RK4()
{
    if(init_time == false){
    t0 = ros::Time::now().toSec();
    current_time = ros::Time::now().toSec() - t0;
    previous_time = current_time - 0.01;
    init_time = true;
    }
    
    current_time = ros::Time::now().toSec() - t0;

    dt = current_time - previous_time;
    rk4.do_step(system_dynamcis,s,current_time,dt);
    cout<<"Time : "<<current_time<<"  Height : "<<s(5)<<endl;
    cout<<"Thrust : "<<T<<endl;

    previous_time = current_time;
}

void Coax_sim::publish_state()
{
    state_.header.stamp = ros::Time::now();
    state_.twist.twist.linear.x = s(0);
    state_.twist.twist.linear.y = s(1);
    state_.twist.twist.linear.z = s(2);

    state_.pose.pose.position.x = s(3);
    state_.pose.pose.position.y = s(4);
    state_.pose.pose.position.z = s(5);

    state_.twist.twist.angular.x = s(6);
    state_.twist.twist.angular.y = s(7);
    state_.twist.twist.angular.z = s(8);
    
    state_.pose.pose.orientation.x = s(9);
    state_.pose.pose.orientation.y = s(10);
    state_.pose.pose.orientation.z = s(11);
    state_.pose.pose.orientation.w = s(12);

    state_publisher.publish(state_);
}

Coax_sim::~Coax_sim()
{

}