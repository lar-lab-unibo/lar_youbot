/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   YouBot.cpp
 * Author: daniele de gregorio
 * 
 * Created on May 6, 2016, 2:05 PM
 */

#include "YouBot.h"
#include <stdio.h>
#include <vector>
#include <kdl/frames.hpp>

namespace lar_youbot {

    YouBot::YouBot() {
        this->init();
    }

    YouBot::YouBot(const YouBot& orig) {
        this->init();
    }

    YouBot::~YouBot() {
    }

    /**
     * Initialize Youbot parameters.
     */
    void YouBot::init() {
        //Wheels
        this->wheels_speed.push_back(0.0); // Front Right
        this->wheels_speed.push_back(0.0); // Front Left
        this->wheels_speed.push_back(0.0); // Back Left
        this->wheels_speed.push_back(0.0); // Back Right

        //Current position
        this->current_position.push_back(0.0);
        this->current_position.push_back(0.0);
        this->current_position.push_back(0.0);
        this->current_position.push_back(0.0);
        this->current_position.push_back(0.0);

        //Signs
        this->signs_for_homing.push_back(1.0);
        this->signs_for_homing.push_back(1.0);
        this->signs_for_homing.push_back(-1.0);
        this->signs_for_homing.push_back(1.0);
        this->signs_for_homing.push_back(1.0);

        //Limits
        //min
        this->limits_min.push_back(DEGtoRAD(-169.0));
        this->limits_min.push_back(DEGtoRAD(-65));
        this->limits_min.push_back(DEGtoRAD(-151.0));
        this->limits_min.push_back(DEGtoRAD(-102));
        this->limits_min.push_back(DEGtoRAD(-167.5));
        //max
        this->limits_max.push_back(DEGtoRAD(169.0));
        this->limits_max.push_back(DEGtoRAD(90.0));
        this->limits_max.push_back(DEGtoRAD(146.0));
        this->limits_max.push_back(DEGtoRAD(102.5));
        this->limits_max.push_back(DEGtoRAD(167.5));

        //Home position
        for (int i = 0; i < LAR_YOUBOT_ARM_JOINTS; i++) {
            this->home_position.push_back(-this->limits_min[i] * this->signs_for_homing[i]);
        }

        //LINKS
        this->H_joint_1 = 0.246; //Height of joint 1 reference frame from the floor
        this->H_base_fk = 0.084; //Not sure, mismatch between SIMULATOR and REAL ROBOT
        this->H_arm_fk = 0.08; //Not sure, mismatch between SIMULATOR and REAL ROBOT
        this->W_la = 0.158;
        this->W_lb = 0.228;
        this->W_R = 0.05;
        this->L = 0.17; //ORIGINAL 0.143 //Distance between center of the mobile base and the base of the arm
        this->L1 = 0.075;
        this->L1_a = 0.033;
        this->L2 = 0.155;
        this->L3 = 0.135;
        this->L4 = 0.218;
        this->L5 = 0.0;

        //REDUNDANCY
        this->redundancy_p1 = 0.3;
        this->redundancy_p2 = 1;
        this->redundancy_p2 = 0.0;

        //initialization
        this->createJointPositionVoidMessage(true);
    }

    /**
     * Creates an JointPositions message for Youbot arm command. Each JointValue of JointPositions message has to be
     * filled with the names of each single Arm (like arm_joint_X) and a "unit" (like Radians). While creating message the method
     * stores it in a member value for caching it.
     * 
     * @param populate_with_home_position TRUE if position is initialized to home position, FLASE if it is initialized to Zero
     * @return brics_actuator::JointPositions message
     */
    brics_actuator::JointPositions YouBot::createJointPositionVoidMessage(bool populate_with_home_position) {

        brics_actuator::JointPositions arm_position;
        std::stringstream ss;
        for (int i = 0; i < LAR_YOUBOT_ARM_JOINTS; i++) {
            brics_actuator::JointValue joint_value;
            ss.str("");
            ss << LAR_YOUBOT_ARM_BASE_NAME << (i + 1);
            joint_value.joint_uri = ss.str();
            if (populate_with_home_position) {
                joint_value.value = this->home_position[i];
            } else {
                joint_value.value = 0;
            }

            joint_value.unit = boost::units::to_string(boost::units::si::radians);
            arm_position.positions.push_back(joint_value);
        }
        current_position_message = arm_position;
        return arm_position;
    }

    /**
     * Fills member brics_actuator::JointPositions message with actual Joint Configuration and returns it
     * @return brics_actuator::JointPositions message
     */
    brics_actuator::JointPositions YouBot::getJointPositionMessage() {
        for (int i = 0; i < LAR_YOUBOT_ARM_JOINTS; i++) {
            current_position_message.positions[i].value = this->home_position[i] + this->current_position[i];
        }
        return current_position_message;
    }

    /**
     * Sets current Joints Position
     * @param joints_position std::vector<double> representing current arm positions
     */
    void YouBot::setCurrentArmPosition(std::vector<double>& joints_position) {
        for (int i = 0; i < LAR_YOUBOT_ARM_JOINTS; i++) {
            this->current_position[i] = joints_position[i];
        }
    }

    /**
     * Sets current Joints Position
     * @param j1 Joint 1
     * @param j2 Joint 2
     * @param j3 Joint 3
     * @param j4 Joint 4
     * @param j5 Joint 5
     */
    void YouBot::setCurrentArmPosition(double j1, double j2, double j3, double j4, double j5) {
        if (this->current_position.size() < LAR_YOUBOT_ARM_JOINTS)return;
        this->current_position[0] = j1;
        this->current_position[1] = j2;
        this->current_position[2] = j3;
        this->current_position[3] = j4;
        this->current_position[4] = j5;
    }

    /**
     * This method is useful to transform Thetas values computed analytically to real actuation values
     * @param theta1 Theta1
     * @param theta2 Theta2
     * @param theta3 Theta3
     * @param theta4 Theta4
     * @param theta5 Theta5
     * @param out_joints OUTPUT vector of joint values.
     * @return 
     */
    void YouBot::transformThetasToJoints(double theta1, double theta2, double theta3, double theta4, double theta5, std::vector<double>& out_joints) {
        out_joints.clear();
        out_joints.push_back(-theta1);
        out_joints.push_back(theta2 >= 0 ? (M_PI / 2.0) - theta2 : -(theta2 - (M_PI / 2.0)));
        out_joints.push_back(theta3 >= 0 ? M_PI - theta3 : -(M_PI + theta3));
        out_joints.push_back(theta4 >= 0 ? -(M_PI - theta4) : (M_PI + theta4));
        out_joints.push_back(-theta5);
    }

    /**
     * Builds a KDL Frames chain (no Denavit-Hartenberg convention) of joints zero positions
     * @param joints_position target configuration of arm joints
     * @param arm_base arm base transform. Optional parameter, leave it void for Identity
     * @return std::vector<KDL::Frame> representing each joint transformation. The last is the EE transform
     */
    std::vector<KDL::Frame> YouBot::getArmChain(const std::vector<double>& joints_position, KDL::Frame arm_base) {
        std::vector<KDL::Frame> frames;

        KDL::Frame A01(KDL::Rotation::RotZ(-joints_position[0]), KDL::Vector(0.0, 0, 0.0));
        KDL::Frame A12(KDL::Rotation::RotX(M_PI / 2.0) * KDL::Rotation::RotZ(-joints_position[1]), KDL::Vector(this->L1_a, 0, this->L1));
        KDL::Frame A23(KDL::Rotation::RotZ(-joints_position[2]), KDL::Vector(0.0, this->L2, 0.0));
        KDL::Frame A34(KDL::Rotation::RotZ(-joints_position[3]), KDL::Vector(0.0, this->L3, 0.0));
        KDL::Frame A45(KDL::Rotation::RotX(-M_PI / 2.0) * KDL::Rotation::RotZ(-joints_position[4]), KDL::Vector(0.0, this->L3, 0.0));

        frames.push_back(arm_base * A01);
        frames.push_back(arm_base * A01 * A12);
        frames.push_back(arm_base * A01 * A12 * A23);
        frames.push_back(arm_base * A01 * A12 * A23 * A34);
        frames.push_back(arm_base * A01 * A12 * A23 * A34 * A45);

        return frames;
    }

    /**
     * Computes a Forward Kinematics based on an arm positions set.
     * @param joints_position arm positions
     * @param ee OUTPUT parameter. Contains the EE transform
     * @return std::vector<KDL::Frame> representing each joint transformation. The last is the EE transform
     */
    std::vector<KDL::Frame> YouBot::fk(const std::vector<double>& joints_position, KDL::Frame& ee) {
        std::vector<KDL::Frame> frames = getArmChain(joints_position);
        ee = frames[frames.size() - 1];
        return frames;
    }

    /**
     * Computes the full Forward Kinematics considering also the mobile base (position and orientation)
     * @param x X position of the mobile base
     * @param y Y position of the mobile base
     * @param theta Theta orientation of the mobile base
     * @param joints_position arm positions
     * @param ee OUTPUT parameter. Contains the EE transform
     * @return std::vector<KDL::Frame> representing base transformation, arm base transformation and each joint transformation. The last is the EE transform
     */
    std::vector<KDL::Frame> YouBot::fk(double x, double y, double theta, const std::vector<double>& joints_position, KDL::Frame& ee) {

        KDL::Frame base(KDL::Rotation::RotZ(theta), KDL::Vector(x, y, H_base_fk));
        KDL::Frame arm_base(KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(L, 0.0, H_arm_fk)));
        arm_base = base*arm_base;

        std::vector<KDL::Frame> frames = getArmChain(joints_position, arm_base);
        ee = frames[frames.size() - 1];
        frames.insert(frames.begin(), base);
        frames.insert(frames.begin(), arm_base);

        return frames;
    }

    /**
     * Sets the redundancy parameters
     * @param p1 Angle between base and arm plane
     * @param p2 Distance between arm base and target. Large values let the IK fails
     * @param p3 "1" if Elbow UP, "-1" if Elbow DOWN
     */
    void YouBot::setRedundacyParameters(double p1, double p2, double p3) {
        this->redundancy_p1 = p1;
        this->redundancy_p2 = p2 > 0.0 ? p2 : 0.0;
        ;
        this->redundancy_p3 = p3 / fabs(p3);
    }

    /**
     * Computes the Inverse Kinematics of whole robot
     * @param target Target Transform for EE
     * @param x OUTPUT, x position of mobile base
     * @param y OUTPUT, y position of mobile base
     * @param theta_out OUTPUT, theta orientation of mobile base
     * @param joints_position OUTPUT, joints positions
     * @return TRUE if IK has solution, FALSE otherwise (Solution is simply checked with NAN numbers in values)
     */
    bool YouBot::ik(KDL::Frame& target, double& x, double& y, double& theta_out, std::vector<double>& joints_position) {

        double beta = 0.0, theta = 0.0, theta1 = 0.0, theta2 = 0.0, theta3 = 0.0, theta4 = 0.0, theta5 = 0.0;

        //Computes Beta and Theta
        double r31 = target(0, 2);
        double r32 = target(1, 2);
        double r33 = target(2, 2);
        beta = atan2(r33, sqrt(pow(r31, 2) + pow(r32, 2)));
        theta = atan2(r32, r31);

        //Computes Wrist distances from arm base
        double Z_armbase = this->H_joint_1;
        double Z_target = target.p.z();
        double Z1 = Z_target - Z_armbase;
        double Z2 = Z1 - this->L4 * sin(beta);
        double X1 = this->redundancy_p2; // P2 for distance between base of the arm and target
        double X2 = X1 - this->L4 * cos(beta);

        //Computes Theta 3
        double cosTheta3 = (-Z2 * Z2 - X2 * X2 + L2 * L2 + L3 * L3) / (2 * L2 * L3);
        double sinTheta3 = sqrt(1 - cosTheta3 * cosTheta3);
        theta3 = atan2(this->redundancy_p3*sinTheta3, cosTheta3); // P3 1 or -1 for ELBOW solutions

        //Computes Theta 2
        double k1 = L2 - L3 * cos(theta3);
        double k2 = L3 * sin(theta3);
        double r = sqrt(k1 * k1 + k2 * k2);
        double lambda = atan2(k2, k1);
        theta2 = atan2(Z2, X2) + lambda;

        //Computes Theta 4
        theta4 = theta2 + theta3 - beta;

        //Computes temporary EE position
        std::vector<double> temp_joints;
        this->transformThetasToJoints(theta, theta2, theta3, theta4, 0.0, temp_joints);
        KDL::Frame ee;
        this->fk(temp_joints, ee);

        //Computes Theta 5
        double e11 = ee.M.Inverse()(0, 0);
        double e21 = ee.M.Inverse()(0, 1);
        double e31 = ee.M.Inverse()(0, 2);

        double e12 = ee.M.Inverse()(1, 0);
        double e22 = ee.M.Inverse()(1, 1);
        double e32 = ee.M.Inverse()(1, 2);

        double g11 = target(0, 0);
        double g21 = target(1, 0);
        double g31 = target(2, 0);

        double cosTheta5 = e11 * g11 + e21 * g21 + e31*g31;
        double sinTheta5 = e12 * g11 + e22 * g21 + e32*g31;
        theta5 = atan2(sinTheta5, cosTheta5);

        //Computes Mobile Base position
        double Xf = ee.p.x();
        double Yf = ee.p.y();

        double Xg = target.p.x();
        double Yg = target.p.y();

        double Xa = Xg - Xf;
        double Ya = Yg - Yf;

        double P1 = this->redundancy_p1;
        double Xp = Xa - L * cos(theta - P1);
        double Yp = Ya - L * sin(theta - P1);

        //Output
        x = Xp;
        y = Yp;
        theta_out = theta - P1;
        theta1 = P1; //Arm Joint1 Correction
        this->transformThetasToJoints(theta1, theta2, theta3, theta4, theta5, joints_position);

        return !(isnan(beta) || isnan(theta) || isnan(theta1) || isnan(theta2) || isnan(theta3) || isnan(theta4) || isnan(theta5) || isnan(x) || isnan(y));
    }

    /**
     * Gets joints home position
     * @return 
     */
    const std::vector<double> &YouBot::getHomePosition() {
        return this->home_position;
    }

    /**
     * Gets joints current position
     * @return 
     */
    const std::vector<double>& YouBot::getCurrentPosition() {
        return this->current_position;
    }

    /**
     * Sets the wheels instant speed. DOESN'T move the base, only used for dynamics computations
     * @param w1 Front Right Wheel
     * @param w2 Front Left Wheel
     * @param w3 Back Left Wheel
     * @param w4 Back Right Wheel
     */
    void YouBot::setWheelsInstantSpeed(double w1, double w2, double w3, double w4) {
        this->wheels_speed[0] = w1;
        this->wheels_speed[1] = w2;
        this->wheels_speed[2] = w3;
        this->wheels_speed[3] = w4;
    }

    /**
     * Gets the instant speed of the base in Robot Coordinate frame
     * @param x X speed
     * @param y Y speed
     * @param theta Theta speed
     */
    void YouBot::getBaseInstantSpeed(double& x, double& y, double& theta) {
        double w1 = this->wheels_speed[0];
        double w2 = this->wheels_speed[1];
        double w3 = this->wheels_speed[2];
        double w4 = this->wheels_speed[3];

        double L = this->W_la + this->W_lb;
        double coeff = this->W_R / (L * 4);
        x = coeff * (L * w1 + L * w2 + L * w3 + L * w4);
        y = coeff * (L * w1 - L * w2 + L * w3 - L * w4);
        theta = coeff * (w1 - w2 - w3 + w4);
    }



}
