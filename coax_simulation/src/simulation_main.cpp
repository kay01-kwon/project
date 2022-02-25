#include <coax_simulation/Coax_sim.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Coax_sim");
    setup_global_variables();
    Coax_sim coax_sim;
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        coax_sim.do_RK4();
        coax_sim.publish_state();
        loop_rate.sleep();

    }
    

}