#include <coax_control/coax_taxing.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Taxing");

    CoaxTAX taxing;
    ros::Rate loop_rate(100);


    while (ros::ok())
    {
        taxing.PublishActuation();
        ros::spinOnce();
        loop_rate.sleep();

    }
    
}