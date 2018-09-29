#include <iostream>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>


namespace Bebop_Ctrl{
class bebop_vel_ctrl{
    public:
        bebop_vel_ctrl(ros::NodeHandle& nh,ros::NodeHandle& pnh);

        void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void VelocityCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void Limitator(double& vx, double& vy, double& vz);
    
    private:
        ros::NodeHandle _nh_;
        ros::Subscriber _get_cmd_vel;
        ros::Subscriber _get_velocity;
        ros::Publisher  _bebop_cmd_vel;

        geometry_msgs::PointStamped _vel_current;
        geometry_msgs::Twist _cmd_vel_pub, _cmd_vel_sub;

        bool _debug;

        double  _K_p_x, _K_p_y, _K_p_z;
        double  _MaxV_xy, _MaxV_z;

};
}






