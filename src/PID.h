#ifndef PID_H
#define PID_H

#include <iostream>
#include <vector>
#include <limits>
#include <cstddef>
#include <iostream>

class PID {
 public:

  int step;
  int update_period;

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double UpdateControl();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Controller initialization
  */
  bool is_initialized;

  /*
  * Parameter optimization
  */
  int gain_index;
  int select_case;
  double current_error;
  double total_error;
  double best_error;
  std::vector<double> K;
  std::vector<double> dK;

};

#endif  // PID_H