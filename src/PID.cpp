#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  p_error = 0;
  i_error = 0;
  d_error = 0;
  Kp = Kp_;
  Kd = Kd_;
  Ki = Ki_;
}

void PID::UpdateError(double cte) {
    double filter_coeff = 0.9;
    d_error = (cte - p_error)*filter_coeff + d_error*(1-filter_coeff);
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
  //cout << "Kp*e: " << -Kp*p_error << " Ki*e: " << -Ki*i_error << " Kd*e: " << -Kd*d_error << " p_e: " << p_error << " i_e: " << i_error  << " d_e: " << d_error << endl;
  return -Kp*p_error - Ki*i_error - Kd*d_error;

}

