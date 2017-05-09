#include "PID.h"
#include <iostream>

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
    d_error = cte_previous - cte_;

    std::cout<<"p_error: "<<p_error<<endl; 
    std::cout<<"d_error: "<<d_error<<endl;
    std::cout<<"i_error: "<<i_error<<endl;
    return; 

}

double PID::TotalError() {
    proportional = -Kp_ * p_error;
    differential = -Kd_ * d_error;
    integral = -Ki_ * i_error;

    std::cout<<"proportional: "<<proportional<<endl; 
    std::cout<<"differential: "<<differential<<endl;
    std::cout<<"integral: "<<integral<<endl;

    return proportional + differential + integral; 
}

// void PID::twiddle() {
//     // every time it runs it executes one optimizing loop like the python algorithm below

//     // -----------  python twiddle loop (trun's pid tutorial) ----------------
//     // while sum(dp) > tol:
//     //   for i in range(len(p)):   --> mode 0
//     //     p[i] += dp[i]
//     //     _, _, err = run(robot, p) --> mode 1
//     //     if err < best_err:
//     //         best_err = err
//     //         dp[i] *= 1.1
//     //     else:
//     //         p[i] -= 2.0 * dp[i]
//     //         _, _, err = run(robot, p) --> mode 2
//     //         if err < best_err:
//     //             best_err = err
//     //             dp[i] *= 1.1
//     //        else:
//     //             p[i] += dp[i]
//     //             dp[i] *= 0.9
//     // --------------------------------------------------

//     if (twiddle_mode_ == 3 || calibration_on_ == false) { // threshold reached or calibration is not active
//         calibrationOFF();
        
//         cout << str_name_ << "*************OFF*************" << endl;
//         return; // no action required
//     }

//     // cout << str_name_ << "  PID Error: " << TotalError() << ", Best Error: " << twiddle_bestError_ << ", iteration: " << twiddle_iterations_ << endl;

//     if (integration_iterations_ < activate_twiddler_every_n_iterations_/2 ) {
//         twiddle_error_ = 0.0;
//         return; // the first half iterations are skipped
//     }
//     else {
//         twiddle_error_ += p_error_ * p_error_; // cte*cte
//     }

//     if (twiddler_yield_ == false) {
//         return; // not time to run twiddler yet
//     }

//     if (twiddle_mode_ == 0) { // beggining of the loop
//         p_[twiddle_optimizingIndex_] += dp_[twiddle_optimizingIndex_];
//         UpdateCoefficients(p_[0], p_[1], p_[2]); // update coefficients
//         twiddle_mode_ = 1; // mode == 1 means increasing dp_
//         return;
//     }

//     if (twiddle_mode_ == 1) { // increase mode
//         if (twiddle_error_ < twiddle_bestError_) {
//             twiddle_bestError_ = twiddle_error_;
//             dp_[twiddle_optimizingIndex_] *= 1.1;
//         }
//         else {
//             p_[twiddle_optimizingIndex_] -= 2 * dp_[twiddle_optimizingIndex_]; // p+dp-2dp = p-dp
//             UpdateCoefficients(p_[0], p_[1], p_[2]); // update coefficients
//             twiddle_mode_ = 2; // mode == 2 means decreasing dp_
//             return;
//         }
//     }
//     else if (twiddle_mode_ == 2) { // twiddle_mode_ == 2
//         if (twiddle_error_ < twiddle_bestError_) {
//             twiddle_bestError_ = twiddle_error_;
//             dp_[twiddle_optimizingIndex_] *= 1.1;
//         }
//         else {
//             p_[twiddle_optimizingIndex_] += dp_[twiddle_optimizingIndex_]; // p+dp-2dp+dp = p
//             UpdateCoefficients(p_[0], p_[1], p_[2]); // update coefficients
//             dp_[twiddle_optimizingIndex_] *= 0.9;
//         }
//     }

//     double sum = std::accumulate(dp_.begin(), dp_.end(), 0.);

//     if (sum < PID::dp_tolerance_ && twiddle_iterations_ > PID::max_iterations_) {
//         twiddle_mode_ = 3; // signals stop twiddling 
//     }
//     else {
//         twiddle_mode_ = 0; // signals next loop
//         twiddle_optimizingIndex_ = (twiddle_optimizingIndex_ + 1) % 3;
//         twiddle_iterations_++;
//     }

// }

