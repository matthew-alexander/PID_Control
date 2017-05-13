#ifndef PID_H
#define PID_H
#include <vector>
#include <uWS/uWS.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  double cte_ ; 
  double cte_previous;
  double sum_cte_;  
  double total_error; 

  double proportional; 
  double differential; 
  double integral; 
  std::vector<double> CTE_errors_;



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
  * Calculate the total PID error.
  */
  double TotalError();

  void twiddle();
   
  
};

#endif /* PID_H */
