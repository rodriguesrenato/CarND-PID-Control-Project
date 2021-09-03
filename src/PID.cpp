#include "PID.h"

#include <iomanip>
#include <iostream>

PID::PID() {}

PID::~PID() {}

// Initialize PID coefficients
void PID::Init(double kp, double ki, double kd, int buffer_size) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  integral_buffer_size = buffer_size;
  i_error_ = 0;
}

// Update PID errors based on cte.
void PID::UpdateError(double cte) {
  // For integral term, it was implemented in two ways and the following is used
  // as the best for this simulation

  // Integral error is a sum of the integral_buffer_size last cte values stored
  // at cte_int_, and when there is a zero crossing on the cte, then the
  // cte_int_ buffer is clear.
  if (cte > 0 && p_error_ < 0 || cte < 0 && p_error_ > 0) {
    cte_int_.clear();
  }
  cte_int_.push_back(cte);
  if (cte_int_.size() > integral_buffer_size) {
    cte_int_.erase(cte_int_.begin());
  }
  i_error_ = 0;
  for (auto i : cte_int_) {
    i_error_ += i;
  }

  // Standard way for the integral term
  // i_error_ += cte;

  d_error_ = cte - p_error_;
  p_error_ = cte;
}

// Calculate and return the total error
double PID::TotalError() {
  return -(kp_ * p_error_ + ki_ * i_error_ + kd_ * d_error_);
}

void PID::Print() {
  std::cout << "\tTotal Error: " << std::setw(8) << TotalError()
            << " | Err: " << std::setw(8) << p_error_ << ", " << std::setw(8)
            << i_error_ << ", " << std::setw(8) << d_error_
            << " | K * Err: " << std::setw(8) << (-kp_ * p_error_) << ", "
            << std::setw(8) << (-ki_ * i_error_) << ", " << std::setw(8)
            << (-kd_ * d_error_);
}