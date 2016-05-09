/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PIDController.cpp
 * Author: daniele
 * 
 * Created on May 9, 2016, 5:10 PM
 */

#include "PIDController.h"
#include <stdio.h>
#include <math.h>

namespace lar_youbot {

    PIDController::PIDController(double Kp, double Kd, double Ki) {
        this->Kp = Kp;
        this->Kd = Kd;
        this->Ki = Ki;
        this->reset();
    }

    PIDController::PIDController(const PIDController& orig) {
        this->Kp = orig.Kp;
        this->Kd = orig.Kd;
        this->Ki = orig.Ki;
        this->error = orig.error;
        this->past_error = orig.past_error;
        this->integral_error = orig.integral_error;
        this->time = orig.time;
    }

    PIDController::~PIDController() {
    }

    /**
     * Updates the Controller
     * @param current_error current error value
     * @param current_time_in_seconds current system time in seconds
     */
    void PIDController::update(double current_error, double current_time_in_seconds) {
        double delta_time = current_time_in_seconds - this->time;
        this->time = current_time_in_seconds;
        this->error = current_error;

        //Proportional
        double proportional = this->error*Kp;

        //Derivative
        double derivative = (this->past_error - this->error) * Kd / delta_time;
        this->past_error = this->error;

        //Integral
        double integral = this->integral_error * Ki*delta_time;
        this->integral_error += this->error;

        //Computes action
        this->actuation = proportional + derivative + integral;

        //NaN Checks
        this->integral_error = isnan(this->integral_error) ? 0.0 : this->integral_error;
        this->past_error = isnan(this->past_error) ? 0.0 : this->past_error;
        this->actuation = isnan(this->actuation) ? 0.0 : this->actuation;
    }

    /**
     * Returns the current Actuation
     * @return 
     */
    double PIDController::getActuation() {
        return this->actuation;
    }

    /**
     * Resets the PID Controller 
     */
    void PIDController::reset() {
        this->error = 0.0;
        this->past_error = 0.0;
        this->integral_error = 0.0;
        this->time = 0.0;
    }


}
