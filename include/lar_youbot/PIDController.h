/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PIDController.h
 * Author: daniele
 *
 * Created on May 9, 2016, 5:10 PM
 */

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
namespace lar_youbot {

    class PIDController {
    public:
        PIDController(double Kp, double Kd, double Ki);
        PIDController(const PIDController& orig);
        virtual ~PIDController();
        double Kp;
        double Kd;
        double Ki;
        
        void update(double current_error, double current_time_in_seconds);
        double getActuation();
        void reset();
    protected:
            
        double time;
        
        double error;
        double past_error;
        double integral_error;
        
        double actuation;
    };
}
#endif /* PIDCONTROLLER_H */

