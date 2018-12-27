#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>
#include <math.h>
#include "json.hpp"
#include <iostream>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double total_error;
  double best_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  // Choose an initialization parameter vector
  float p[3] = {0, 0, 0};
  // Define potential changes
  float dp[3] = {1, 1, 1};
  unsigned int para_index;

  /*
  * Twiddle 
  */ 
  double min_ctrl_value;
  double max_ctrl_value;
  bool Kp_init_done;
  bool Ki_init_done;
  bool Kd_init_done;
  bool is_twiddle_init_done;
  bool is_twiddle_done;
  unsigned int twiddle_step;
  unsigned long twiddle_counter;
  unsigned long twiddle_counter_threshold;
  double twiddle_sum_threshold;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Controller for PID.
  */
  double Controller();

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Used to reset the simulator and bring the car back to first position.
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);

  /*
  * Twiddle or vanilla gradient descent for tuning one hyper parameter at a time
  */
  void Twiddle();

  /*
  * Initial twiddle
  */
  void Twiddle_Init();
};

#endif /* PID_H */
