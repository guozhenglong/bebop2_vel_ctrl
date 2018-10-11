#include <iostream>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>


namespace Bebop_Ctrl{
class bebop_pos_ctrl{
    public:
        bebop_pos_ctrl(ros::NodeHandle& nh,ros::NodeHandle& pnh);
        void Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Point &euler);
        void currentPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void commandPositionCallback(const geometry_msgs::Twist::ConstPtr& msg);
    private:
        ros::NodeHandle _nh_;
        ros::Subscriber _get_current_pos;
        ros::Subscriber _get_command_pos;
        ros::Publisher  _velocity_cmd;
        geometry_msgs::PoseStamped _pos_current;
        double _yaw;
        geometry_msgs::Twist _command_pos;
        geometry_msgs::Twist _velocity_cmd_pub;

        bool _debug;

        double  _K_p_x, _K_p_y, _K_p_z;

};
}






