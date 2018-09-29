#include "bebop_vel_ctrl/bebop_vel_ctrl.h"

namespace Bebop_Ctrl
{
    bebop_vel_ctrl::bebop_vel_ctrl(ros::NodeHandle& nh,ros::NodeHandle& pnh):_nh_(nh)
    {  
        pnh.param("debug", _debug, true); 
        pnh.param("K_p_x", _K_p_x, 0.2); 
        pnh.param("K_p_y", _K_p_y, 0.2); 
        pnh.param("K_p_z", _K_p_z, 0.2); 
        pnh.param("MaxV_xy", _MaxV_xy, 0.3); 
        pnh.param("MaxV_z", _MaxV_z, 0.2); 

        _bebop_cmd_vel = _nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);
        _get_cmd_vel = _nh_.subscribe("/velocity_cmd",2,&bebop_vel_ctrl::CmdVelCallback,this);
        _get_velocity = _nh_.subscribe("/velocity_current",2,&bebop_vel_ctrl::VelocityCallback,this);
        usleep(50000);  // 10000ms can not receive correct data
        ros::spin();

    }


    void bebop_vel_ctrl::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        _cmd_vel_sub = *msg;
        _cmd_vel_pub.linear.x = _K_p_x*(_cmd_vel_sub.linear.x - _vel_current.point.x);
        _cmd_vel_pub.linear.y = _K_p_y*(_cmd_vel_sub.linear.y - _vel_current.point.y);
        _cmd_vel_pub.linear.z = _K_p_z*(_cmd_vel_sub.linear.z - _vel_current.point.z);
        _cmd_vel_pub.angular = _cmd_vel_sub.angular;
        bebop_vel_ctrl::Limitator(_cmd_vel_pub.linear.x, _cmd_vel_pub.linear.y, _cmd_vel_pub.linear.z);
        _bebop_cmd_vel.publish(_cmd_vel_pub);
    }

    void bebop_vel_ctrl::VelocityCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        _vel_current = *msg;
    }

    void bebop_vel_ctrl::Limitator(double& vx, double& vy, double& vz)
    {
        if(vx < (- _MaxV_xy))
            vx = - _MaxV_xy;
        if(vx > _MaxV_xy)
            vx = _MaxV_xy;

        if(vy < (- _MaxV_xy))
            vy = - _MaxV_xy;
        if(vy > _MaxV_xy)
            vy = _MaxV_xy; 

        if(vz < (- _MaxV_z))
            vz = - _MaxV_z;
        if(vz > _MaxV_z)
            vz = _MaxV_z;  
    }

}