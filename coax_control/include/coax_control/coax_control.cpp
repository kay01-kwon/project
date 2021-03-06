#include "coax_control.h"

CoaxCTRL::CoaxCTRL()
{
    is_init_pos = false;
    init_yaw = 0;
    // Constructor
    cout<<"*****Mass Parameter Setup*****"<<endl;
    nh.getParam("mass",mass);
    cout<<"Mass (kg): ";
    cout<<mass<<endl;
    I_W_CM << 0, 0, mass*g;

    cout<<"*****COM Offset Parameter Setup*****"<<endl;
    nh.getParam("CM_x_T",CM_x_T);
    nh.getParam("CM_y_T",CM_y_T);
    nh.getParam("CM_z_T",CM_z_T);

    double phi,theta;

    CM_p_CM_T << CM_x_T, CM_y_T, CM_z_T;
    CM_u_CM_T = CM_p_CM_T.normalized();
    cout<<CM_p_CM_T<<endl;

    phi = -asin(CM_u_CM_T(1));
    theta = atan2(CM_u_CM_T(0)/cos(phi),CM_u_CM_T(2)/cos(phi));

    eq_rp << phi/2, theta; // Roll, pitch (y-x convention)
    hovering_rp = -eq_rp;
    cout<<"*****Equlibrium TVC Info (deg)*****"<<endl;
    cout<<"Roll : ";
    cout<<eq_rp(0)*180.0/M_PI<<endl;
    cout<<"Pitch : ";
    cout<<eq_rp(1)*180.0/M_PI<<endl;

    cout<<"*****Hovering Attitude Info (deg)*****"<<endl;
    cout<<"Roll : ";
    cout<<hovering_rp(0)*180.0/M_PI<<endl;
    cout<<"Pitch : ";
    cout<<hovering_rp(1)*180.0/M_PI<<endl;

    cout<<"*****Get Position Gain Parameter*****"<<endl;
    
    nh.getParam("Kp_pos",Kp_pos);
    nh.getParam("Kd_pos",Kd_pos);

    cout<<"Kp_pos Gain: "<<Kp_pos<<endl;
    cout<<"Kd_pos Gain: "<<Kd_pos<<endl;

    nh.getParam("Kp_ori_phi",Kp_ori_phi);
    nh.getParam("Kp_ori_theta",Kp_ori_theta);
    nh.getParam("Kp_ori_yaw",Kp_ori_yaw);
    
    nh.getParam("Kd_ori_phi",Kd_ori_phi);
    nh.getParam("Kd_ori_theta",Kd_ori_theta);
    nh.getParam("Kd_ori_yaw",Kd_ori_yaw);


    cout<<"Kp_ori phi Gain: "<<Kp_ori_phi<<endl;
    cout<<"Kp_ori theta Gain: "<<Kp_ori_theta<<endl;
    cout<<"Kp_ori yaw Gain: "<<Kp_ori_yaw<<endl;

    //cout<<"Kd_ori Gain: "<<Kd_ori_phi<<endl;

    cout<<"*****Get Lift Parameter*****"<<endl;
    nh.getParam("C_lift",C_lift);
    cout<<"C lift : "<<C_lift<<endl;

    cout<<"*****Subscriber Setup*****"<<endl;

    cout<<"Desired Trajectory Subscriber Setup"<<endl;

    traj_subscriber = nh.subscribe("/des_traj",1,&CoaxCTRL::CallbackDesTraj,this);

    cout<<"Estimated Pose Subscriber Setup"<<endl;
    pose_subscriber = nh.subscribe("/mavros/odometry/in",1,&CoaxCTRL::CallbackPose,this);

    cout<<"*****Publisher Setup*****"<<endl;
    cout<<"Actuator Publisher Setup"<<endl;
    actuator_publisher = nh.advertise<actuator>("/actuation",1);
    odom_publisher = nh.advertise<Odometry>("/Test",1);

    I_p_CM.setZero();
    I_v_CM.setZero();

    I_q_CM.setZero();
    I_q_CM(0) = 1;
    I_w.setZero();

    I_a_des.setZero();
    I_v_des.setZero();
    I_p_des.setZero();

    I_q_des.setZero();
    I_q_des(0) = 1;

    roll_pitch_yaw.setZero();
    rpy_error.setZero();
    
    des_roll_pitch_yaw.setZero();

    idx = 1;
    size_of_mv_avg = 20;

}

void CoaxCTRL::CallbackDesTraj(const traj & des_traj)
{

    I_a_des << des_traj.Accel.linear.x,
            des_traj.Accel.linear.y,
            des_traj.Accel.linear.z;

    I_v_des << des_traj.Twist.linear.x,
            des_traj.Twist.linear.y,
            des_traj.Twist.linear.z;

    I_p_des << des_traj.Pose.position.x, 
            des_traj.Pose.position.y, 
            des_traj.Pose.position.z;
    
    I_q_des << des_traj.Pose.orientation.w,
            des_traj.Pose.orientation.x,
            des_traj.Pose.orientation.y,
            des_traj.Pose.orientation.z;


}

void CoaxCTRL::CallbackPose(const Odometry & pose_msg)
{

    I_p_CM << pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y,
            pose_msg.pose.pose.position.z;
 
    I_q_CM << pose_msg.pose.pose.orientation.w,
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z;
    
    I_v_CM << pose_msg.twist.twist.linear.x,
            pose_msg.twist.twist.linear.y,
            pose_msg.twist.twist.linear.z;
    
    if (idx < size_of_mv_avg ){
        wx.push_back(pose_msg.twist.twist.angular.x);
        wy.push_back(pose_msg.twist.twist.angular.y);
        wz.push_back(pose_msg.twist.twist.angular.z);
        idx++;
    }
    else{
        idx = 10;
        wx.erase(wx.begin(),wx.begin()+10);
        wy.erase(wy.begin(),wy.begin()+10);
        wz.erase(wz.begin(),wz.begin()+10);
    }


    I_w << std::accumulate(wx.begin(),wx.end(),0.0)/(double)size_of_mv_avg,
    	std::accumulate(wy.begin(),wy.end(),0.0)/(double)size_of_mv_avg,
        std::accumulate(wz.begin(),wz.end(),0.0)/(double)size_of_mv_avg;

    roll_pitch_yaw(0) = asin(2*(qy*qz + qw*qx));
    
    roll_pitch_yaw(1) = -atan2(2*(qx*qz - qw*qy)/cos(roll_pitch_yaw(0)),
    (1-2*(pow(qx,2)+pow(qy,2)))/cos(roll_pitch_yaw(0)));

    roll_pitch_yaw(2) = atan2(2*(qx*qy - qw*qz)/cos(roll_pitch_yaw(0)),
    (1-2*(pow(qx,2)+pow(qz,2)))/cos(roll_pitch_yaw(0)))
     - init_yaw;
    
    if(is_init_pos == false)
    {
        I_p_CM_init = I_p_CM;
        init_yaw = roll_pitch_yaw(2);
        R <<cos(init_yaw), -sin(init_yaw), 0,
            sin(init_yaw), cos(init_yaw), 0,
            0, 0, 1;
        is_init_pos = true;
    }

    I_p_CM = R*(I_p_CM - I_p_CM_init);
    yaw = roll_pitch_yaw(2);
    roll_pitch_yaw(2) = atan2(sin(yaw),cos(yaw));
    I_v_CM = RotM(roll_pitch_yaw(2))*I_v_CM;
    cout<<roll_pitch_yaw*180.0/M_PI<<endl;


    if(is_init_pos = true)
    {
        //PosControl();
        OriControl();
        odom_data.header.stamp = ros::Time::now();
        odom_data.pose.pose.position.x = I_p_CM(0);
        odom_data.pose.pose.position.y = I_p_CM(1);
        odom_data.pose.pose.position.z = I_p_CM(2);

        odom_data.twist.twist.linear.x = I_v_CM(0);
        odom_data.twist.twist.linear.y = I_v_CM(1);
        odom_data.twist.twist.linear.z = I_v_CM(2);
        
        odom_data.twist.twist.angular.x = I_w(0);
        odom_data.twist.twist.angular.y = I_w(1);
        odom_data.twist.twist.angular.z = I_w(2);	

        odom_publisher.publish(odom_data);
    }


}

void CoaxCTRL::PosControl()
{
    u_pos = (I_a_des
            + Kp_pos*(I_p_des - I_p_CM) 
            + Kd_pos*(I_v_des - I_v_CM)) + I_W_CM;
    
    //thrust = sqrt(u_pos.transpose()*u_pos);
    //throttle = thrust;
    
    //throttle = sqrt(thrust / gear_ratio / C_lift);
    //
    if (u_pos(0) > 10)
        u_pos(0) = 10;
    else if (u_pos(0) < -10)
        u_pos(0) = -10;

    if (u_pos(1) > 10)
        u_pos(1) = 10;
    else if (u_pos(1) < -10)
        u_pos(1) = -10;

    throttle_clamping(throttle);

    des_yaw = -2*atan2(qz_des,qw_des);
    des_yaw = 0;
    thrust = 588.6;

    if(thrust > 0){
        des_roll_pitch_yaw(0) = asin((u_pos(0)*sin(des_yaw)+u_pos(1)*cos(des_yaw))/thrust) + hovering_rp(0);
        des_roll_pitch_yaw(1) = asin((u_pos(0)*cos(des_yaw)-u_pos(1)*sin(des_yaw))/thrust) + hovering_rp(1);
        des_roll_pitch_yaw(2) = des_yaw;
    }
    else{
        des_roll_pitch_yaw << 0, 0, des_yaw;
    }

    des_rp_clamping(des_roll_pitch_yaw(0),des_roll_pitch_yaw(1));

}

void CoaxCTRL::OriControl()
{
    rpy_error = des_roll_pitch_yaw - roll_pitch_yaw;

    yaw_clamping();

    //u_att = Kp_ori * (des_roll_pitch_yaw - roll_pitch_yaw) - Kd_ori*I_w;
    u_att(0) = Kp_ori_phi*(des_roll_pitch_yaw(0) - roll_pitch_yaw(0)) - Kd_ori_phi*I_w(0);
    u_att(1) = Kp_ori_theta*(des_roll_pitch_yaw(1) - roll_pitch_yaw(1)) - Kd_ori_theta*I_w(1);
    u_att(2) = Kp_ori_yaw * (des_roll_pitch_yaw(2) - roll_pitch_yaw(2)) + Kd_ori_yaw*I_w(2);
    u_att(0) -= eq_rp(0);
    u_att(1) -= eq_rp(1);

    //cout<<roll_pitch_yaw*180.0/M_PI<<endl;
    //cout<<endl;

    actuator_data.main_rotor_pwm = throttle;
    actuator_data.phi_u =  -u_att(0);
    actuator_data.theta_u = - u_att(1);

    u_att(2) = u_att(2) * 100.0;
    if( u_att(2) > 80.0 )
	    u_att(2) = 80.0;
    else if(u_att(2) < -80.0)
	    u_att(2) = -80.0;

    actuator_data.left_servo = u_att(2);
    actuator_data.right_servo = u_att(2);

    cout<<"Yaw input: "<<actuator_data.left_servo<<endl;

    actuator_publisher.publish(actuator_data);

}

void CoaxCTRL::throttle_clamping(uint16_t &throttle_ptr)
{
    if (throttle_ptr > throttle_max)
        throttle_ptr = throttle_max;
    
    if(throttle_ptr < throttle_min)
        throttle_ptr = throttle_min;

}

void CoaxCTRL::des_rp_clamping(double &des_roll_ptr, double &des_pitch_ptr)
{
    if (abs(des_roll_ptr) > des_roll_max)
        des_roll_ptr = des_roll_max * signum(des_roll_ptr);

    if (abs(des_pitch_ptr) > des_pitch_max)
        des_pitch_ptr = des_pitch_max * signum(des_pitch_ptr);
}

double CoaxCTRL::signum(double &sign_ptr)
{
    if (sign_ptr > 0)
        return 1.0; 
    if (sign_ptr < 0)
        return -1.0;
    return 0.0;
}

void CoaxCTRL::yaw_clamping()
{
    if (des_roll_pitch_yaw(2) > M_PI_2 && roll_pitch_yaw(2) < - M_PI_2)
        rpy_error(2) = rpy_error(2) - 2*M_PI;
    else if (des_roll_pitch_yaw(2) < -M_PI_2 && roll_pitch_yaw(2) > M_PI_2)
        rpy_error(2) = rpy_error(2) + 2*M_PI;
}

Matrix3d CoaxCTRL::RotM(double &yaw)
{
    Matrix3d R;
    R << cos(yaw), sin(yaw), 0,
        -sin(yaw), cos(yaw), 0,
        0, 0, 1;

    return R;
}


CoaxCTRL::~CoaxCTRL()
{

}
