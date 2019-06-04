#include "PID.h"
#include <iostream>
using namespace std;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;

	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;

	d_p = 1;
	d_i = 1;
	d_d = 1;

	iterate_counts = 0;	
	max_cte = 4;
	error = 0;
	best_error = 10 * max_cte*max_cte;
	

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	double pre_cte = p_error;

	p_error = cte;	
	i_error += cte;	
	d_error = cte - pre_cte;		

	iterate_counts++;
	if (iterate_counts > ITERATE_N) {
		error += cte * cte;
	}

}

double PID::TotalError() {
	/**
	* TODO: Calculate and return the total error
	*/
	//return 0.0;  // TODO: Add your total error calc here!

	double pid_error = -Kp * p_error - Ki * i_error - Kd * d_error;

#if 0
	if ((d_p + d_i + d_d) < TOLERANCE) { //twiddle finished
		return pid_error;
	}


	if (abs(p_error) > max_cte && iterate_counts > 10) { 
		pid_error = std::numeric_limits<double>::quiet_NaN();
		Twiddle();
	}
	else if (iterate_counts > 2 * ITERATE_N) { 
		pid_error = std::numeric_limits<double>::quiet_NaN();
		Twiddle();
	}

	if (iterate_counts > 2 * ITERATE_N) {
		pid_error = std::numeric_limits<double>::quiet_NaN();
		Twiddle();
	}
#endif
	return pid_error;

	
}


void PID::Twiddle() {
	static int state_pid = 0;
	static bool b_increase = true;
	double err = 0;
	if (iterate_counts <= ITERATE_N) {
		err = max_cte * max_cte;
	}
	else {
		err = error * ITERATE_N / (iterate_counts*(iterate_counts - ITERATE_N));
	}
	//p
	if (0 == state_pid) {
		state_pid = 1;
		if (b_increase) {
			if (err < best_error) {
				best_error = err;
				d_p *= 1.1;
			}
			else {
				Kp -= 2 * d_p;
				b_increase = false;
			}
		}
		else {
			if (err < best_error) {
				best_error = err;
				d_p *= 1.1;
			}
			else {
				Kp += d_p;
				d_p *= 0.9;				
			}
			b_increase = true;

		}
	}
	//i
	else if (1 == state_pid) {
		state_pid = 2;
		if (b_increase) {
			if (err < best_error) {
				best_error = err;
				d_i *= 1.1;
			}
			else {
				Ki -= 2 * d_i;
				b_increase = false;
			}
		}
		else {
			if (err < best_error) {
				best_error = err;
				d_i *= 1.1;
			}
			else {
				Ki += d_i;
				d_i *= 0.9;
			}
			b_increase = true;

		}
	}
	//d
	else if (2 == state_pid) {
		state_pid = 0;
		if (b_increase) {
			if (err < best_error) {
				best_error = err;
				d_d *= 1.1;
			}
			else {
				Kd -= 2 * d_d;
				b_increase = false;
			}
		}
		else {
			if (err < best_error) {
				best_error = err;
				d_d *= 1.1;
			}
			else {
				Kd += d_d;
				d_d *= 0.9;
			}
			b_increase = true;

		}
	}

	error = 0;
	iterate_counts = 0;

	p_error = 0;
	i_error = 0;
	d_error = 0;

}

bool PID::IsNeedTwiddle() {

	return b_need_twiddle;
}

bool PID::IsTwiddleFinished() {

	return b_twiddle_finished;
}

bool PID::IsTwiddling() {

	return b_twiddling;
}

void PID::NeedTwiddle(bool is_need) {
	b_need_twiddle = is_need;
}
void PID::IsTwiddleFinished(bool is_finished) {
	b_twiddle_finished = is_finished;
}
void PID::IsTwiddling(bool is_twiddling) {
	b_twiddling = is_twiddling;
}