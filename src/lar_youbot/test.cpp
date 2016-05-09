#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_kdl.h"

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>


ros::Publisher pub;
ros::Subscriber sub;
ros::Subscriber sub_cube;
ros::Publisher twist_pub;
geometry_msgs::Twist twist;

double x_current,y_current,theta_current;
double x_setpoint,y_setpoint,theta_setpoint;
double x_dot,y_dot,theta_dot;
double x_error,y_error,theta_error;

double P_x = 1;
double P_y = P_x;
double P_theta = 0.1;

double D_x = 1;
double D_y = 1;
double D_theta = 1;

double I_x = 1;
double I_y = 1;
double I_theta = 1;

void cube_pos_cb(const geometry_msgs::PoseStamped& msg){
    x_setpoint = msg.pose.position.x;
    y_setpoint = msg.pose.position.y;
}

void youbot_pos_cb(const geometry_msgs::PoseStamped& msg){

    geometry_msgs::Pose pose = msg.pose;

    KDL::Rotation r = KDL::Rotation::Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    KDL::Vector p(
        pose.position.x,
        pose.position.y,
        pose.position.z
    );
    KDL::Frame frame(r,p);

    x_current = p.x();
    y_current = p.y();

    double roll,pitch,yaw;
    frame.M.GetRPY(roll,pitch,yaw);

    std::cout << "POS:"<<std::endl;

    std::cout << frame.p.x() <<std::endl;
    std::cout << frame.p.y() <<std::endl;
    std::cout << yaw <<std::endl<<std::endl;

    theta_current = yaw;
/*
    std::cout << "ROT:" <<std::endl;

    std::cout << roll <<std::endl;
    std::cout << pitch <<std::endl;
    std::cout << yaw <<std::endl<<std::endl;
*/

}

int
main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "my_pcl_tutorial");
        ros::NodeHandle nh;

    //Topics
    sub = nh.subscribe ("/vrep/youbotOdom", 1, youbot_pos_cb);
    sub_cube = nh.subscribe ("/vrep/cubeOdom", 1, cube_pos_cb);
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //Target
        //x_setpoint = 2;
    //y_setpoint = -1;
    theta_setpoint = 0;
        // Spin
    ros::Rate r(100);
    double time = 0.0f;
        while(nh.ok() ) {


            //x_setpoint = 1*sin(time*0.001);
        //y_setpoint = 1*cos(time*0.001);
        theta_setpoint = 0;


x_error = x_setpoint - x_current;
        y_error = y_setpoint - y_current;
        theta_error = theta_setpoint - theta_current;

        x_dot = x_error*P_x;
        y_dot = y_error*P_y;
        theta_dot = theta_error*P_theta;


        double angle = theta_current;
        double x_dot_youbot = x_dot * cos(angle) + y_dot * sin(angle);
        double y_dot_youbot = x_dot * -sin(angle) + y_dot * cos(angle);
            //std::cout << "Vx:" << x_dot_youbot<<std::endl;
            //std::cout << "Vy:" << y_dot_youbot<<std::endl;
        twist.linear.x = x_dot_youbot; //speed
        twist.linear.y = y_dot_youbot; //speed
        twist.angular.z = theta_dot; //angle

        twist_pub.publish(twist);
                ros::spinOnce();
        r.sleep();
        time++;
        }
}

