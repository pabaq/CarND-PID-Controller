#include <iostream>
#include <algorithm>
#include <cmath>
#include "PID.h"

// PID controller
PID::PID(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) {
  this->p_error = 0;
  this->d_error = 0;
  this->i_error = 0;
}

PID::~PID() {}

// Update PID errors based on the cross track error.
void PID::update_error(double cte) {
  double d_error = (cte - this->p_error);
  // Since the simulator sometimes sends weird error jumps between iteration
  // steps, the derivative errors will kind of be filtered, to avoid strong 
  // controller reactions on this error jumps
  if (std::fabs(d_error) < 0.15) { 
    this->d_error = d_error;  // derivative error
  }
  this->i_error += cte;  // integral error
  this->p_error = cte;  // proportional error
}

// Calculate and return the total error
double PID::total_error() {
  double perr = kp * p_error;
  double ierr = ki * i_error;
  double derr = kd * d_error;
  double total_error = perr + ierr + derr;
  printf("PID Components:%10.4f%8.4f%8.4f\n", -perr, -ierr, -derr);
  return total_error;  
}