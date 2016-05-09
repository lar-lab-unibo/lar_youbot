/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   YouBot.cpp
 * Author: daniele
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

    void YouBot::init() {

        //Current position
        this->current_position.push_back(0.0);
        this->current_position.push_back(0.0);
        this->current_position.push_back(0.0);
        this->current_position.push_back(0.0);
        this->current_position.push_back(0.0);

        //Signs
        this->signs.push_back(1.0);
        this->signs.push_back(1.0);
        this->signs.push_back(-1.0);
        this->signs.push_back(1.0);
        this->signs.push_back(1.0);

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
            this->home_position.push_back(-this->limits_min[i] * this->signs[i]);
        }

        //LINKS
        this->L = 0.17;
        this->L1 = 0.075;
        this->L1_a = 0.033;
        this->L2 = 0.155;
        this->L3 = 0.135;
        this->L4 = 0.218;
        this->L5 = 0.0;

        //Transforms
        this->T_floor_base = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.2));
        this->T_base_armbase = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.143, 0.0, 0.046));
        this->T_floor_armbase = this->T_floor_base * this->T_base_armbase;

        //init
        this->createJointPositionVoidMessage(true);
    }

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

    brics_actuator::JointPositions YouBot::getJointPositionMessage() {
        for (int i = 0; i < LAR_YOUBOT_ARM_JOINTS; i++) {
            current_position_message.positions[i].value = this->home_position[i] + this->current_position[i];
        }
        return current_position_message;
    }

    void YouBot::setCurrentPosition(std::vector<double>& joints_position) {
        for (int i = 0; i < LAR_YOUBOT_ARM_JOINTS; i++) {
            this->current_position[i] = joints_position[i];
        }

    }

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

    std::vector<KDL::Frame> YouBot::fk(const std::vector<double>& joints_position, KDL::Frame& ee) {
        std::vector<KDL::Frame> frames = getArmChain(joints_position);
        ee = frames[frames.size() - 1];
        return frames;
    }

    std::vector<KDL::Frame> YouBot::fk(double x, double y, double theta, const std::vector<double>& joints_position, KDL::Frame& ee) {

        KDL::Frame base(KDL::Rotation::RotZ(theta), KDL::Vector(x, y, 0.0953));
        KDL::Frame arm_base(KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(L, 0.0, 0.08)));
        arm_base = base*arm_base;

        std::vector<KDL::Frame> frames = getArmChain(joints_position, arm_base);
        ee = frames[frames.size() - 1];
        frames.insert(frames.begin(), base);
        frames.insert(frames.begin(), arm_base);

        return frames;
    }

    void YouBot::updateChain(double theta1, double theta2, double theta3, double theta4, double theta5) {

        this->current_position[0] = theta1;
        this->current_position[1] = theta2;
        this->current_position[2] = theta3;
        this->current_position[3] = theta4;
        this->current_position[4] = theta5;


    }

    void YouBot::setBaseTransform(KDL::Frame frame) {

        this->base_transform = frame;
    }

    bool YouBot::computeBetaTheta(KDL::Frame& target, double& beta, double& theta) {
        double r31 = target(0, 2);
        double r32 = target(1, 2);
        double r33 = target(2, 2);
        beta = atan2(r33, sqrt(pow(r31, 2) + pow(r32, 2)));
        theta = atan2(r32, r31);
        return true;
    }

    bool YouBot::computeElbow(KDL::Frame& target, double& theta, double& theta1, double& theta2, double& theta3, double& theta4) {
        double beta;
        if (this->computeBetaTheta(target, beta, theta)) {
            theta1 = 0.0;
            double Z_armbase = this->T_floor_armbase.p.z();
            double Z_target = target.p.z();
            double Z1 = Z_target - Z_armbase;
            double Z2 = Z1 - this->L4 * sin(beta);
            //printf("Z1,Z2= %f,%f\n", Z1, Z2);
            double X1 = 0.35; //P2
            double X2 = X1 - this->L4 * cos(beta);
            
            printf("theta %f\n", theta);
            printf("beta %f\n", beta);
            
            
            //theta 3
            double cosTheta3 = (-Z2 * Z2 - X2 * X2 + L2 * L2 + L3 * L3) / (2 * L2 * L3);
            double sinTheta3 = sqrt(1 - cosTheta3 * cosTheta3); // If P3 then multiply it for -1
            theta3 = atan2(sinTheta3, cosTheta3);
            //printf("c3,s3= %f,%f\n", cosTheta3, sinTheta3);
            
            
            //theta 2
            double k1 = L2 - L3 * cos(theta3);
            double k2 = L3 * sin(theta3);
            double r = sqrt(k1 * k1 + k2 * k2);
            double lambda = atan2(k2, k1);
            theta2 = atan2(Z2, X2) + lambda;

            //theta 4
            theta4 = theta2 + theta3 - beta;

           
            return true;
        }
        return false;

    }

    bool YouBot::computeEEOrientation(KDL::Frame& target, KDL::Frame& ee, double& theta5) {
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
        if(fabs(theta5-M_PI)<0.01 || fabs(theta5+M_PI)<0.01){
            theta5 = 0.0;
        }
    }

    bool YouBot::computeEE(KDL::Frame& target, double& theta5) {

        double e11 = this->armbase_ee.M(0, 0);
        double e21 = this->armbase_ee.M(0, 1);
        double e31 = this->armbase_ee.M(0, 2);

        double e12 = this->armbase_ee.M(1, 0);
        double e22 = this->armbase_ee.M(1, 1);
        double e32 = this->armbase_ee.M(1, 2);

        double g11 = target(0, 0);
        double g21 = target(1, 0);
        double g31 = target(2, 0);


        double cosTheta5 = e11 * g11 + e21 * g21 + e31*g31;
        double sinTheta5 = e12 * g11 + e22 * g21 + e32*g31;
        theta5 = atan2(sinTheta5, cosTheta5);
    }

    bool YouBot::computeIK(KDL::Frame& target, double& x, double& y, double& theta, double& theta1, double& theta2, double& theta3, double& theta4, double& theta5) {

        computeElbow(target, theta,theta1, theta2, theta3, theta4);
        printf("Elbow %f %f %f %f\n", theta1, theta2, theta3, theta4);

        std::vector<double> temp_joints;
        temp_joints.push_back(-theta);
        temp_joints.push_back((M_PI/2.0)-theta2);
        temp_joints.push_back(M_PI-theta3);
        temp_joints.push_back(-(M_PI-theta4));
        temp_joints.push_back(0.0);
        
        KDL::Frame ee;
        this->fk(temp_joints,ee);
        printf("EE: %f, %f, %f\n",ee.p.x(),ee.p.y(),ee.p.z());
        this->computeEEOrientation(target,ee,theta5);
        
        theta = theta;
        double Xf = ee.p.x();
        double Yf = ee.p.y();
        printf("Xf,Yf= %f,%f\n", Xf, Yf);
        double Xg = target.p.x();
        double Yg = target.p.y();
        printf("Xg,Yg= %f,%f\n", Xg, Yg);
        double Xa = Xg - Xf;
        double Ya = Yg - Yf;
        printf("Xa,Ya= %f,%f\n", Xa, Ya);
        
        double P3 = 0;
        double Xp = Xa - L * cos(theta - P3);
        double Yp = Ya - L * sin(theta - P3);
        printf("Xp,Yp= %f,%f\n", Xp, Yp);
        x = Xp;
        y = Yp;

        theta1 = 0;
        theta = theta - P3;

    }

    const std::vector<double> &YouBot::getHomePosition() {

        return this->home_position;
    }

    const std::vector<double>& YouBot::getCurrentPosition() {
        return this->current_position;
    }





}
