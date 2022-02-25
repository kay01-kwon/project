#include <coax_control/coax_control.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"Coax_control");

    CoaxCTRL coax_control;

    while (ros::ok())
    {
        ros::spinOnce();


    }
    
}