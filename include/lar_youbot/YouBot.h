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

namespace lar_youbot {

    class YouBot {
    public:
        YouBot();
        YouBot(const YouBot& orig);
        virtual ~YouBot();
        void setBaseTransform(KDL::Frame frame);
        bool computeBetaTheta(KDL::Frame& target, double& beta, double& theta);
        bool computeEEOrientation(KDL::Frame& target,KDL::Frame& ee, double& theta5);
        bool computeElbow(KDL::Frame& target, double& theta, double& theta1,double& theta2, double& theta3, double& theta4);
        bool computeEE(KDL::Frame& target, double& theta5);
        bool computeIK(KDL::Frame& target, double&x, double&y, double& theta, double& theta1, double& theta2, double& theta3, double& theta4, double& theta5);
        void updateChain(double theta1, double theta2, double theta3, double theta4, double theta5);

        brics_actuator::JointPositions createJointPositionVoidMessage(bool populate_with_home_position = true);
        brics_actuator::JointPositions getJointPositionMessage();

        std::vector<double> const &getHomePosition();


        std::vector<double> const &getCurrentPosition();
        void setCurrentPosition(std::vector<double>& joints_position);

        //
        std::vector<KDL::Frame> getArmChain(const std::vector<double>& joints_position,KDL::Frame arm_base = KDL::Frame::Identity());
        std::vector<KDL::Frame> fk(const std::vector<double>& joints_position, KDL::Frame& ee);
        std::vector<KDL::Frame> fk(double x, double y, double theta, const std::vector<double>& joints_position, KDL::Frame& ee);
        
        //DEBUG
        KDL::Frame TEMP;
    protected:
        void init();

        double L;
        double L1;
        double L1_a;
        double L2;
        double L3;
        double L4;
        double L5;

        KDL::Frame link0_link1;
        KDL::Frame link1_link2;
        KDL::Frame link2_link3;
        KDL::Frame link3_link4;
        KDL::Frame link4_link5;
        KDL::Frame armbase_ee;
        

        KDL::Frame base_transform;
        KDL::Frame T_floor_base;
        KDL::Frame T_base_armbase;
        KDL::Frame T_floor_armbase;

        brics_actuator::JointPositions current_position_message;
        std::vector<double> current_position;
        std::vector<double> limits_min;
        std::vector<double> limits_max;
        std::vector<double> home_position;
        std::vector<double> signs;

    };
}

#endif /* YOUBOT_H */

