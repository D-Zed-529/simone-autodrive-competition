#ifndef PID_H
#define PID_H

#include <iostream>

class PIDController {
public:
    void configure(double kp, double ki, double kd, double dt);

    void setPID(double kp, double ki, double kd, double dt);

    void setTarget(double setpoint);

    void Setpoint(double setpoint);

    void setLimits(double minOutput, double maxOutput);

    void setOutputLimits(double minOutput, double maxOutput);

    double update(double input);

    double compute(double input);

    void printTarget() const;

    void Print();

private:
    double kp;
    double ki;
    double kd;
    double dt;
    double setpoint;
    double minOutput;
    double maxOutput;
    double integralTerm;
    double lastError;
};

inline void PIDController::configure(double kp, double ki, double kd, double dt) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
    this->setpoint = 0.0;
    this->minOutput = 0.0;
    this->maxOutput = 1.0;
    this->integralTerm = 0.0;
    this->lastError = 0.0;
}

inline void PIDController::setPID(double kp, double ki, double kd, double dt) {
    configure(kp, ki, kd, dt);
}

void PIDController::setTarget(double setpoint) {
    this->setpoint = setpoint;
}

void PIDController::Setpoint(double setpoint) {
    setTarget(setpoint);
}

void PIDController::setLimits(double minOutput, double maxOutput) {
    this->minOutput = minOutput;
    this->maxOutput = maxOutput;
}

void PIDController::setOutputLimits(double minOutput, double maxOutput) {
    setLimits(minOutput, maxOutput);
}

double PIDController::update(double input) {
    double error = setpoint - input;
    double proportionalTerm = kp * error;
    integralTerm += ki * error * dt;
    double derivativeTerm = kd * (error - lastError) / dt;
    lastError = error;

    double output = proportionalTerm + integralTerm + derivativeTerm;
    if (output < minOutput) {
        output = minOutput;
    } else if (output > maxOutput) {
        output = maxOutput;
    }

    return output;
}

double PIDController::compute(double input) {
    return update(input);
}

void PIDController::printTarget() const {
    std::cout<<"targetspeed=="<< double(this->setpoint) << std::endl;
}

void PIDController::Print() {
    printTarget();
}

#endif 
