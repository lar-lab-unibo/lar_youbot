/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   YouBot.h
 * Author: daniele
 *
 * Created on May 6, 2016, 2:05 PM
 */

#ifndef YOUBOT_H
#define YOUBOT_H

#include <kdl/frames.hpp>

#include <brics_actuator/JointPositions.h>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#define LAR_YOUBOT_ARM_JOINTS 5
#define LAR_YOUBOT_ARM_BASE_NAME "arm_joint_"

#define DEGtoRAD(angleDegrees) (angleDegrees * M_PI / 180.0)
#define RADtoDEG(angleRadians) (angleRadians * 180.0 / M_PI)
#define ISNAN(value) (value!=value)

namespace lar_youbot {

    class YouBot {
    public:
        YouBot();
        YouBot(const YouBot& orig);
        virtual ~YouBot();
        
        
        //Ros messages management
        brics_actuator::JointPositions createJointPositionVoidMessage(bool populate_with_home_position = true);
        brics_actuator::JointPositions getJointPositionMessage();

        //Home Position
        std::vector<double> const &getHomePosition();

        //Current Position
        std::vector<double> const &getCurrentPosition();
        void setCurrentArmPosition(std::vector<double>& joints_position);
        void setCurrentArmPosition(double j1, double j2, double j3, double j4, double j5);
        
        //Redundancy set
        void setRedundacyParameters(double p1,double p2,double p3);
        
        //FK
        std::vector<KDL::Frame> getArmChain(const std::vector<double>& joints_position, KDL::Frame arm_base = KDL::Frame::Identity());
        std::vector<KDL::Frame> fk(const std::vector<double>& joints_position, KDL::Frame& ee);
        std::vector<KDL::Frame> fk(double x, double y, double theta, const std::vector<double>& joints_position, KDL::Frame& ee);
        //IK
        bool ik(KDL::Frame& target, double&x, double&y, double& theta, std::vector<double>& joints_position);

        //UTILS
        void transformThetasToJoints(double theta1, double theta2, double theta3, double theta4, double theta5, std::vector<double>& out_joints);

        //DEBUG
        KDL::Frame TEMP;
        std::vector<KDL::Frame> TEMP_V;
    protected:
        void init();

        //Geometry of the robot
        double H_joint_1;
        double H_base_fk;
        double H_arm_fk;
        double L;
        double L1;
        double L1_a;
        double L2;
        double L3;
        double L4;
        double L5;

        //Redundancy parameters
        double redundancy_p1;
        double redundancy_p2;
        double redundancy_p3;

        //Robot status
        brics_actuator::JointPositions current_position_message;
        std::vector<double> current_position;
        std::vector<double> limits_min;
        std::vector<double> limits_max;
        std::vector<double> home_position;
        std::vector<double> signs_for_homing;

    };
}

#endif /* YOUBOT_H */

