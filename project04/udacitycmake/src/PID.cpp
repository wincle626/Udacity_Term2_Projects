#include "PID.h"

using namespace std;
using json = nlohmann::json;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    p_error = 0;
    i_error = 0;
    d_error = 0;

    best_error = 9999;
    total_error = 0;
    min_ctrl_value = 99999;
    max_ctrl_value = -99900;

    twiddle_counter = 0;
    twiddle_counter_threshold = 100;
    twiddle_sum_threshold = 0.001;
  
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;
    dp[0] = Kp/3;
    dp[1] = Ki/4;
    dp[2] = Kd/2;
    para_index = 0;

    is_twiddle_done = false;
    twiddle_step = 0;
    Kp_init_done = false;
    Ki_init_done = false;
    Kd_init_done = false;
    is_twiddle_init_done = false;

}

void PID::UpdateError(double cte) {

	double previous_cte = p_error;

	p_error = cte;
	i_error += cte;
	d_error = cte - previous_cte;

	total_error += cte*cte;
	twiddle_counter += 1;

	if(cte < min_ctrl_value){
		min_ctrl_value = cte;
	} 

	if(cte > max_ctrl_value){
		max_ctrl_value = cte;
	} 

}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

#define PCTRL 1
#define ICTRL 1
#define DCTRL 1

double PID::Controller() {

#if PCTRL==1 && ICTRL==1 && DCTRL==1
	return - Kp*p_error - Ki*i_error - Kd*d_error;
#elif PCTRL==1 && ICTRL==1
	return - Kp*p_error - Ki*i_error;
#elif PCTRL==1 && DCTRL==1
	return - Kp*p_error - Kd*d_error;
#elif IDCTRL==1 && DCTRL==1
	return - Ki*i_error - Kd*d_error;
#elif PCTRL==1
	return - Kp*p_error;
#elif ICTRL==1
	return - Ki*i_error;
#elif DCTRL==1
	return - Kd*d_error;
#endif

}

double PID::TotalError() {

	return total_error/(float)twiddle_counter;

}


void PID::Twiddle_Init(){
	if(!Kp_init_done){
		if(Kp>0){
			Kp += Kp/1000;
		}
		if(abs(min_ctrl_value + max_ctrl_value) < 0.1){
			Kp_init_done = true;
		}
	}
	if(!Kd_init_done){
		if(Kd>0){
			Kd += Kd/1000;
		}		
		if(abs(total_error) < 0.1){
			Kd_init_done = true;
		}
	}
	if(Kp_init_done && Kd_init_done && !Ki_init_done){
		if(Ki>0){
			Ki += Ki/1000;
		}	
	}

	// reset total error to zero
	total_error = 0;
	// reset number of steps to zero because twiddle will need to start again
	twiddle_counter = 0;

	if(Kp_init_done && Kd_init_done && Ki_init_done){
		is_twiddle_init_done = true;
	}
}

void PID::Twiddle() {

	/*
	# Choose an initialization parameter vector
	p = [0, 0, 0]
	# Define potential changes
	dp = [1, 1, 1]
	# Calculate the error
	best_err = A(p)
	threshold = 0.001
	for i in range(len(p)):
		p[i] += dp[i]
		err = A(p)

		if err < best_err:  # There was some improvement
		    best_err = err
		    dp[i] *= 1.1
		else:  # There was no improvement
		    p[i] -= 2*dp[i]  # Go into the other direction
		    err = A(p)

		    if err < best_err:  # There was an improvement
		        best_err = err
		        dp[i] *= 1.05
		    else  # There was no improvement
		        p[i] += dp[i]
		        # As there was no improvement, the step size in either
		        # direction, the step size might simply be too big.
		        dp[i] *= 0.95
	*/

	//get Current error
	double current_error = TotalError();

	// Initiate current best error 
	if(best_error > 999)
	{
		best_error = current_error;
	}
	std::cout << "best_error:" << best_error
		  << " current_error:" << current_error
		  << std::endl;
	std::cout << " total_error:" << total_error
		  << " twiddle_counter:" << twiddle_counter
		  << std::endl;

	// reset total error to zero
	total_error = 0;
	// reset number of steps to zero because twiddle will need to start again
	twiddle_counter = 0;
	switch(twiddle_step){
		case 0:
			p[para_index] += dp[para_index];
			Kp = p[0];
			Ki = p[1];
			Kd = p[2];
			twiddle_step = 1;
			break;
		case 1:
			if(current_error < best_error){				
				best_error = current_error;
				dp[para_index] *= 1.1;
			}else{
				p[para_index] -= 2*dp[para_index];
				Kp = p[0];
				Ki = p[1];
				Kd = p[2];
			}
			twiddle_step = 2;
			break;
		case 2:
			if(current_error < best_error){				
				best_error = current_error;
				dp[para_index] *= 1.05;
			}else{
				p[para_index] += dp[para_index];
				dp[para_index] *= 0.95;
			}
			para_index = (para_index+1)%3;
			twiddle_step = 0;
			break;
		default:
			p[para_index] += dp[para_index];
			break;
			
	}

	if((dp[0]+dp[1]+dp[2]) < twiddle_sum_threshold){
		is_twiddle_done = true;
	}
}

