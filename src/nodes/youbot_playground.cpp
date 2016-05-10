#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_kdl.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <brics_actuator/JointPositions.h>
#include "lar_youbot/YouBot.h"
#include "lar_youbot/PIDController.h"

//Ros interface
ros::Subscriber subscriber_youbot_pose;
ros::Subscriber subscriber_target_pose;
ros::Subscriber subscriber_youbot_joints;
ros::Publisher publisher_mobile_base_vel;
ros::Publisher publisher_arm_joints;
geometry_msgs::Twist mobile_base_twist;

//Youbot
lar_youbot::YouBot youbot;

//Vrep Green Sphere Target
KDL::Frame vrep_target;
KDL::Frame testing_odometry;

//Mobile Base Controls
double x_current, y_current, theta_current;
double x_setpoint, y_setpoint, theta_setpoint;
lar_youbot::PIDController pidX(5.0, 0.1, 0.0);
lar_youbot::PIDController pidY(5.0, 0.1, 0.0);
lar_youbot::PIDController pidTheta(2.1, 0.1, 0.0);

/**
 * Convert geometry_msgs::Pose to KDL::Frame
 * @param pose source geometry_msgs::Pose
 * @param frame target KDL::Frame
 */
void poseToFrame(const geometry_msgs::Pose& pose, KDL::Frame& frame) {
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
    frame = KDL::Frame(r, p);
}

/**
 * Converts KDL::Frame to geometry_msgs::Pose
 * @param frame source KDL::Frame 
 * @param pose target geometry_msgs::Pose
 */
void frameToPose(KDL::Frame& frame, geometry_msgs::Pose& pose) {
    pose.position.x = frame.p.x();
    pose.position.y = frame.p.y();
    pose.position.z = frame.p.z();

    frame.M.GetQuaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
            );
}

/**
 * Joint state callback. Contains also WHEEL feedback
 * @param msg
 */
void jointstate_cb(const sensor_msgs::JointState& msg) {

    //The address in joints array may change in real robot. CHECK!
    double w1 = msg.velocity[7];
    double w2 = msg.velocity[5];
    double w3 = msg.velocity[2];
    double w4 = msg.velocity[0];
    youbot.setWheelsInstantSpeed(w1, w2, w3, w4);
}

/**
 * Callback for Target Pose
 * @param msg
 */
void target_pos_cb(const geometry_msgs::PoseStamped& msg) {
    geometry_msgs::Pose pose = msg.pose;
    poseToFrame(pose, vrep_target);
}

/**
 * Callback for Youbot Pose
 * @param msg
 */
void youbot_pos_cb(const geometry_msgs::PoseStamped& msg) {
    geometry_msgs::Pose pose = msg.pose;
    KDL::Frame frame;
    poseToFrame(msg.pose, frame);

    double roll, pitch, yaw;
    frame.M.GetRPY(roll, pitch, yaw);

    x_current = frame.p.x();
    y_current = frame.p.y();
    theta_current = yaw;
}

/**
 * Main
 * @param argc
 * @param argv
 * @return 
 */
int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "lar_youbot_playground");
    ros::NodeHandle nh;

    //Topics
    subscriber_youbot_pose = nh.subscribe("/youbotPose", 1, youbot_pos_cb);
    subscriber_target_pose = nh.subscribe("/target", 1, target_pos_cb);
    subscriber_youbot_joints = nh.subscribe("/joint_states", 1, jointstate_cb);
    publisher_mobile_base_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    publisher_arm_joints = nh.advertise<brics_actuator::JointPositions>("/arm_controller/position_command", 1);

    //debug
    ros::Publisher debug_publisher = nh.advertise<geometry_msgs::PoseStamped>("/target_debug", 1);
    geometry_msgs::PoseStamped debug_pose_stamped;


    //Test Odometry
    testing_odometry = KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0), KDL::Vector(0.0,0.0,0.4));


    // Spin & Time
    ros::Rate r(100);
    double time;
    ros::Time ros_time = ros::Time::now();
    ros::Duration delta_time = ros::Time::now() - ros_time;
    //Main Loop
    while (nh.ok()) {

        //ROS time
        delta_time = ros::Time::now() - ros_time;
        ros_time = ros::Time::now();

        //Current Time
        time = ros_time.toSec(); // ros::Time().now().toNSec()*0.000000001; // Converts Nanoseconds to Seconds

        //Youbot Redundancy control. Comment those lines to disable moving solution
        double P1 = (M_PI / 2.0) * sin(time * 0.3); // Rotation of the base w.r.t. arm plane. Try to SET always 0
        double P2 = 0.35 + sin(time)*0.1; // Distance between target and the base of the arm. Try to SET always 0
        double P3 = 1; // Elbow UP
        youbot.setRedundacyParameters(P1, P2, P3);


        // Computes IK of Base+Arm
        double x, y, theta;
        std::vector<double> joints;
        bool good_solution = youbot.ik(vrep_target, x, y, theta, joints);
        ROS_INFO("Solution %s", good_solution ? "GOOD" : "FAIL");

        //Arm Setpoint
        if (good_solution) {
            youbot.setCurrentArmPosition(joints);
            publisher_arm_joints.publish(youbot.getJointPositionMessage());
        }

        //Mobile Base Section
        {
            //Mobile base setpoint
            x_setpoint = x;
            y_setpoint = y;
            theta_setpoint = theta - M_PI / 2.0;
            //Computes Mobile Base position error
            double x_error = x_setpoint - x_current;
            double y_error = y_setpoint - y_current;
            double theta_error = theta_setpoint - theta_current;
            //Update Mobile Base PIDs controllers
            pidX.update(x_error, time);
            pidY.update(y_error, time);
            pidTheta.update(theta_error, time);
            //Computes Speed in World Reference Frame
            double x_dot = pidX.getActuation(); // x_error * P_x; // + x_error_integral*delta_time*I_x;
            double y_dot = pidY.getActuation(); // y_error * P_y ; // + y_error_integral*delta_time*I_y;
            double theta_dot = pidTheta.getActuation(); // theta_error * P_theta + (theta_error - theta_error_last) * D_theta / delta_time; // + theta_error_integral*delta_time*I_theta;
            //Computes Speed in Robot Reference Frame
            double angle = theta_current + M_PI / 2;
            double x_dot_youbot = x_dot * cos(angle) + y_dot * sin(angle);
            double y_dot_youbot = x_dot * -sin(angle) + y_dot * cos(angle);
            //Publish Mobile Base Speed
            mobile_base_twist.linear.x = x_dot_youbot; //speed
            mobile_base_twist.linear.y = y_dot_youbot; //speed
            mobile_base_twist.angular.z = theta_dot; //angle
            publisher_mobile_base_vel.publish(mobile_base_twist);


        }

        //Spin loop
        ros::spinOnce();
        r.sleep();
    }
}

