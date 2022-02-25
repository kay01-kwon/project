#include <tvc_control/tvc_control.h>

int main(int argc,char**argv)
{
    ros::init(argc,argv,"Test");
    tvc_test tvc_ex;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}