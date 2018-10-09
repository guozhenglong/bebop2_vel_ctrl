#include "bebop_pos_ctrl/bebop_pos_ctrl.h"

namespace Bebop_Ctrl
{
    bebop_pos_ctrl::bebop_pos_ctrl(ros::NodeHandle& nh,ros::NodeHandle& pnh):_nh_(nh)
    {  
        pnh.param("debug", _debug, true); 
        pnh.param("K_p_x", _K_p_x, 0.2); 
        pnh.param("K_p_y", _K_p_y, 0.2); 
        pnh.param("K_p_z", _K_p_z, 0.2); 
       
        _velocity_cmd = _nh_.advertise<geometry_msgs::Twist>("velocity_cmd",1);
        _get_current_pos = _nh_.subscribe("pos_sync",2,&bebop_pos_ctrl::currentPositionCallback,this);
        _get_command_pos = _nh_.subscribe("target_pos",2,&bebop_pos_ctrl::commandPositionCallback,this);
        usleep(50000);  // 10000ms can not receive correct data
        ros::spin();

    }


    void bebop_pos_ctrl::commandPositionCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        _command_pos = *msg;
        _velocity_cmd_pub.linear.x = _K_p_x*(_command_pos.linear.x - _pos_current.pose.position.x);
        _velocity_cmd_pub.linear.y = _K_p_y*(_command_pos.linear.y - _pos_current.pose.position.y);
        _velocity_cmd_pub.linear.z = _K_p_z*(_command_pos.linear.z - _pos_current.pose.position.z);
        geometry_msgs::Point euler_Angle;
        geometry_msgs::Quaternion quaternion;
        quaternion = _pos_current.pose.orientation;
        Quat2Euler(quaternion, euler_Angle);
        _velocity_cmd_pub.angular.x = 0.0;
        _velocity_cmd_pub.angular.y = 0.0;
        _velocity_cmd_pub.angular.z = -0.5 * euler_Angle.z;
        _velocity_cmd.publish(_velocity_cmd_pub);
    }

    void bebop_pos_ctrl::currentPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        _pos_current = *msg;
    }

    void bebop_pos_ctrl::Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Point &euler)
    {
        double q0 = quat.w;
        double q1 = quat.x;
        double q2 = quat.y;
        double q3 = quat.z;
        double t0 = -2.0 * (q2 * q2 + q3 * q3) + 1.0;
        double t1 = +2.0 * (q1 * q2 + q0 * q3);
        double t2 = -2.0 * (q1 * q3 - q0 * q2);
        double t3 = +2.0 * (q2 * q3 + q0 * q1);
        double t4 = -2.0 * (q1 * q1 + q2 * q2) + 1.0;
        //t2 = t2 > 1.0 ? 1.0 : t2;
        //t2 = t2 < -1.0 ? -1.0 : t2;
        // cout<<"pitch =  :"<< asin(t2)<<endl;
        // cout<<"roll  =  :"<< atan2(t3, t4)<<endl;
        // cout<<"yaw   =  :"<< atan2(t1, t0)<<endl;
        euler.x = asin(t2);
        euler.y = -atan2(t3, t4);
        euler.z = atan2(t1, t0);
    }


}