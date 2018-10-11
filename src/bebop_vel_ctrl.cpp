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

        _bebop_cmd_vel = _nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
        _get_cmd_vel = _nh_.subscribe("velocity_cmd",2,&bebop_vel_ctrl::CmdVelCallback,this);
        _get_velocity = _nh_.subscribe("velocity_current",2,&bebop_vel_ctrl::VelocityCallback,this);
        usleep(50000);  // 10000ms can not receive correct data
        ros::spin();

    }


    void bebop_vel_ctrl::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        _cmd_vel_sub = *msg;
        
        double cmd_x = _K_p_x*(_cmd_vel_sub.linear.x - _vel_current.pose.position.x);
        double cmd_y = _K_p_y*(_cmd_vel_sub.linear.y - _vel_current.pose.position.y);
        double cmd_z = _K_p_z*(_cmd_vel_sub.linear.z - _vel_current.pose.position.z);
        double cmd_yaw = _cmd_vel_sub.angular.z;

        _cmd_vel_pub.linear.x = cmd_x*cos(_yaw) - cmd_y*sin(_yaw);
        _cmd_vel_pub.linear.y = -1.0*(cmd_x*sin(_yaw) + cmd_y*cos(_yaw));
        _cmd_vel_pub.linear.z = cmd_z;
        _cmd_vel_pub.angular.z = -1.0*cmd_yaw;
        bebop_vel_ctrl::Limitator(_cmd_vel_pub.linear.x, _cmd_vel_pub.linear.y, _cmd_vel_pub.linear.z);
        _bebop_cmd_vel.publish(_cmd_vel_pub);
    }

    void bebop_vel_ctrl::VelocityCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        _vel_current = *msg;
        geometry_msgs::Point euler_Angle;
        geometry_msgs::Quaternion quaternion;
        quaternion = _vel_current.pose.orientation;
        Quat2Euler(quaternion, euler_Angle);
        _yaw = -1.0*euler_Angle.z;
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
    void bebop_vel_ctrl::Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Point &euler)
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