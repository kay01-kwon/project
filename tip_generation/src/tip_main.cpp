#include <tip_generation/triple_integral_path.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"TIP_gen");
    TIP tip_gen;
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        tip_gen.traj_publish();
        loop_rate.sleep();
    }
}