#include "bebop_vel_ctrl/bebop_vel_ctrl.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bebop_vel_ctrl_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Bebop_Ctrl::bebop_vel_ctrl Bebop_Vel_Control(nh, pnh);
    return 0;
}
