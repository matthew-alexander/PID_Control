#include "PID.h"
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    Kp_ = Kp; 
    Ki_ = Ki; 
    Kd_ = Kd; 

    // p_error = p_error; 
    // i_error = i_error; 
    // d_error = d_error; 
    // std::cout<<"P_error: "<<p_error;
    sum_cte_ = 0.0; 
  

}

void PID::UpdateError(double cte) {
    cte_previous = cte_; 
    
    cte_ = cte; 
    sum_cte_ += cte_; 
    

    p_error = cte_; 
    i_error = sum_cte_; 
    d_error = cte_ - cte_previous;

    std::cout<<"p_error: "<<p_error<<endl; 
    std::cout<<"d_error: "<<d_error<<endl;
    std::cout<<"i_error: "<<i_error<<endl;

    CTE_errors_.push_back(cte); 
    std::cout << "length of list of errors_: " << CTE_errors_.size() << std::endl;

    return; 

}

double PID::TotalError() {
    proportional = Kp_ * p_error;
    differential = Kd_ * d_error;
    integral = Ki_ * i_error;

    std::cout<<"proportional: "<<proportional<<endl; 
    std::cout<<"differential: "<<differential<<endl;
    std::cout<<"integral: "<<integral<<endl;

    return proportional + differential + integral; 
}




 

