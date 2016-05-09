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
#include <brics_actuator/JointPositions.h>
#include "lar_youbot/YouBot.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

ros::Publisher pub;
ros::Subscriber sub;
ros::Subscriber sub_target;
ros::Subscriber sub_cube;
ros::Publisher twist_pub;
ros::Publisher target_tf_pub;
ros::Publisher arm_pub;
geometry_msgs::Twist twist;

KDL::Frame vrep_target;

bool ready_feedback = false;
double x_current, y_current, theta_current;
double x_setpoint, y_setpoint, theta_setpoint;
double x_dot, y_dot, theta_dot;
double x_error, y_error, theta_error;

double P_x = 1;
double P_y = P_x;
double P_theta = 0.1;

double D_x = 1;
double D_y = 1;
double D_theta = 1;

double I_x = 1;
double I_y = 1;
double I_theta = 1;

void poseToFrame(geometry_msgs::Pose& pose, KDL::Frame& frame) {
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

void cube_pos_cb(const geometry_msgs::PoseStamped& msg) {
    x_setpoint = msg.pose.position.x;
    y_setpoint = msg.pose.position.y;
}

void target_pos_cb(const geometry_msgs::PoseStamped& msg) {
    geometry_msgs::Pose pose = msg.pose;
    poseToFrame(pose, vrep_target);
}

void youbot_pos_cb(const geometry_msgs::PoseStamped& msg) {

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
    KDL::Frame frame(r, p);

    x_current = p.x();
    y_current = p.y();

    double roll, pitch, yaw;
    frame.M.GetRPY(roll, pitch, yaw);

    std::cout << "POS:" << std::endl;

    std::cout << frame.p.x() << std::endl;
    std::cout << frame.p.y() << std::endl;
    std::cout << yaw << std::endl << std::endl;
    
    
//    if(ready_feedback){
//        while(fabs(theta_current-yaw)>M_PI){
//            printf("Last %f   Yaw %f\n",theta_current,yaw);
//            exit(1);
//        }
//    }
    
    printf("Yaw  %f  CompYaw  %f\n",yaw, M_PI*2.0 - yaw);
    theta_current = yaw;//>0? yaw: M_PI*2.0 - yaw;
    ready_feedback=true;
    /*
        std::cout << "ROT:" <<std::endl;

        std::cout << roll <<std::endl;
        std::cout << pitch <<std::endl;
        std::cout << yaw <<std::endl<<std::endl;
     */

}

brics_actuator::JointPositions armFromQs(double theta1, double theta2, double theta3, double theta4, double theta5) {
    brics_actuator::JointPositions arm_position;

    brics_actuator::JointValue v1;
    v1.joint_uri = "arm_joint_1";
    v1.value = 169 * M_PI / 180.0 + 0 + theta1;
    v1.unit = boost::units::to_string(boost::units::si::radians);

    brics_actuator::JointValue v2;
    v2.joint_uri = "arm_joint_2";
    v2.value = 65 * M_PI / 180.0 + 1.57 - theta2;
    v2.unit = boost::units::to_string(boost::units::si::radians);

    brics_actuator::JointValue v3;
    v3.joint_uri = "arm_joint_3";
    v3.value = -151 * M_PI / 180.0 + M_PI - theta3;
    v3.unit = boost::units::to_string(boost::units::si::radians);

    brics_actuator::JointValue v4;
    v4.joint_uri = "arm_joint_4";
    v4.value = +102.5 * M_PI / 180.0 - (M_PI - theta4);
    v4.unit = boost::units::to_string(boost::units::si::radians);

    brics_actuator::JointValue v5;
    v5.joint_uri = "arm_joint_5";
    v5.value = +167.5 * M_PI / 180.0 + (-theta5);
    v5.unit = boost::units::to_string(boost::units::si::radians);


    arm_position.positions.push_back(v1);
    arm_position.positions.push_back(v2);
    arm_position.positions.push_back(v3);
    arm_position.positions.push_back(v4);
    arm_position.positions.push_back(v5);
    return arm_position;
}

int
main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    //Topics
    sub = nh.subscribe("/youbotPose", 1, youbot_pos_cb);
    sub_target = nh.subscribe("/target", 1, target_pos_cb);
    sub_cube = nh.subscribe("/vrep/cubeOdom", 1, cube_pos_cb);
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    target_tf_pub = nh.advertise<geometry_msgs::PoseStamped>("target_tf", 1);
    arm_pub = nh.advertise<brics_actuator::JointPositions>("/arm_controller/position_command", 1);

    ros::Publisher arm_tf_pub_1 = nh.advertise<geometry_msgs::PoseStamped>("/arm_tf_a01", 1);
    ros::Publisher arm_tf_pub_2 = nh.advertise<geometry_msgs::PoseStamped>("/arm_tf_a12", 1);
    ros::Publisher arm_tf_pub_3 = nh.advertise<geometry_msgs::PoseStamped>("/arm_tf_a23", 1);
    ros::Publisher arm_tf_pub_4 = nh.advertise<geometry_msgs::PoseStamped>("/arm_tf_a34", 1);
    ros::Publisher arm_tf_pub_5 = nh.advertise<geometry_msgs::PoseStamped>("/arm_tf_a45", 1);
    geometry_msgs::PoseStamped arm_tf_msg_1;
    geometry_msgs::PoseStamped arm_tf_msg_2;
    geometry_msgs::PoseStamped arm_tf_msg_3;
    geometry_msgs::PoseStamped arm_tf_msg_4;
    geometry_msgs::PoseStamped arm_tf_msg_5;

    geometry_msgs::PoseStamped target_tf_msg;

    lar_youbot::YouBot youbot;

    brics_actuator::JointPositions arm_position = youbot.createJointPositionVoidMessage(true);


    //KDL::Frame target(KDL::Rotation::RPY(0,M_PI/3,M_PI/4), KDL::Vector(1, 0, 0.5));


    //return 0;

    //Target
    //x_setpoint = 2;
    //y_setpoint = -1;
    theta_setpoint = 0;
    // Spin
    ros::Rate r(10);
    double time = 0.0f;
    while (nh.ok()) {
        double time = ros::Time().now().toNSec()*0.0000000005;

        //x_setpoint = -2;
        //y_setpoint = -2;
        //theta_setpoint = -M_PI/2.0;

        KDL::Frame target = vrep_target; //(KDL::Rotation::RPY(0, M_PI / 2.0 + sin(time)*0.2, M_PI * sin(time)), KDL::Vector(sin(time * 0.2), cos(time * 0.15), 0.38 + sin(time)*0.02));
        frameToPose(target, target_tf_msg.pose);
        double x, y, theta, theta1, theta2, theta3, theta4, theta5;
        KDL::Frame target_corrected = target * KDL::Frame(KDL::Rotation::RotZ(M_PI / 2.0), KDL::Vector::Zero());
        youbot.computeIK(target, x, y, theta, theta1, theta2, theta3, theta4, theta5);

        printf("J: %f,%f,%f,%f,%f\n", theta1, theta2, theta3, theta4, theta5);


        x_setpoint = x;
        y_setpoint = y;
        theta_setpoint = theta - M_PI / 2.0;
        //        for (int i = 0; i < LAR_YOUBOT_ARM_JOINTS; i++) {

        std::vector<double> joints;
        joints.push_back(-theta1);
        joints.push_back((M_PI / 2.0) - theta2);
        joints.push_back(M_PI - theta3);
        joints.push_back(-(M_PI - theta4));
        joints.push_back(-theta5);
        youbot.setCurrentPosition(joints);
        arm_position = youbot.getJointPositionMessage();

        // std::cout << (M_PI*sin(time*0.8))*180.0/M_PI<<std::endl;
        //        }
        KDL::Frame ee;
        std::vector<KDL::Frame> frames = youbot.fk(x_current, y_current, theta_current + M_PI / 2.0, youbot.getCurrentPosition(), ee);
        frameToPose(frames[2], arm_tf_msg_1.pose);
        frameToPose(frames[3], arm_tf_msg_2.pose);
        frameToPose(frames[4], arm_tf_msg_3.pose);
        frameToPose(frames[5], arm_tf_msg_4.pose);
        frameToPose(frames[6], arm_tf_msg_5.pose);

        arm_tf_pub_1.publish(arm_tf_msg_1);
        arm_tf_pub_2.publish(arm_tf_msg_2);
        arm_tf_pub_3.publish(arm_tf_msg_3);
        arm_tf_pub_4.publish(arm_tf_msg_4);
        arm_tf_pub_5.publish(arm_tf_msg_5);


        //frameToPose(ee,target_tf_msg.pose);
        //brics_actuator::JointPositions arm_position = // armFromQs(theta1, theta2, theta3, theta4, theta5);

        bool nan = false;
        if (x != x || y != y || theta != theta || theta1 != theta1 || theta2 != theta2 || theta3 != theta3 || theta4 != theta4 || theta5 != theta5) {
            nan = true;
        }

        target_tf_pub.publish(target_tf_msg);

        if (!nan) {
            arm_pub.publish(arm_position);
        }
        //        x_setpoint = x; // 1*sin(time*0.001);
        //        y_setpoint = y; // 1*cos(time*0.001);
        //        theta_setpoint = theta;


        x_error = x_setpoint - x_current;
        y_error = y_setpoint - y_current;
        theta_error = theta_setpoint - theta_current;

        x_dot = x_error*P_x;
        y_dot = y_error*P_y;
        theta_dot = theta_error*P_theta;


        double angle = theta_current + M_PI / 2;
        double x_dot_youbot = x_dot * cos(angle) + y_dot * sin(angle);
        double y_dot_youbot = x_dot * -sin(angle) + y_dot * cos(angle);
        std::cout << "\nAngle:" << angle << std::endl;
        std::cout << "Vx:" << x_dot_youbot << std::endl;
        std::cout << "Vy:" << y_dot_youbot << std::endl;
        twist.linear.x = x_dot_youbot; //speed
        twist.linear.y = y_dot_youbot; //speed
        twist.angular.z = theta_dot; //angle

        if (!nan) {
            twist_pub.publish(twist);
        }
        ros::spinOnce();
        r.sleep();
        time++;
    }
}

