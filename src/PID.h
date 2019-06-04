#ifndef PID_H
#define PID_H

const double TOLERANCE = 0.001;
const int ITERATE_N = 2000;

class PID {
public:
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
	double TotalError();

	void Twiddle();
	bool IsNeedTwiddle();
	bool IsTwiddleFinished();
	bool IsTwiddling();

	void NeedTwiddle(bool is_need);
	void IsTwiddleFinished(bool is_finished);
	void IsTwiddling(bool is_twiddling);

private:
	//
	bool b_need_twiddle;
	bool b_twiddle_finished;
	bool b_twiddling;

	 
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

	// used by twiddle
	double d_p;
	double d_i;
	double d_d;

	//
	int iterate_counts;
	double best_error;
	double error;
	double max_cte;
};

#endif  // PID_H