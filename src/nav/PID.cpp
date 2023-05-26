
#include "PID.h"

unit_t PID::curr_err_sum()
{
    unit_t sum = 0;
    // Iterate back to last term or start
    // Boolean condition sets i = 0 when all terms are desired
    for (int i = std::max(0, (err_sum_terms != -1)*(curr_err_term - err_sum_terms));
             i < curr_err_term;
             i++) {
        sum += err_array[i];
    }
    return sum;
}

unit_t PID::step(unit_t next_err, unit_t time_step)
{
    // Check if array is full
    if (curr_err_term == MAXTIMESTEPS) {
        printf( "PID error: Maximum time steps reached.\n");
        return 0;
    }
    // Add new incoming error to array
    err_array[curr_err_term++] = next_err;
    // Check if time_step is zero
    if (time_step == 0) {
        return 0;
    }
    // Compute and return command
    unit_t u = Kp * next_err +
               Ki * curr_err_sum() * time_step +
               Kd * (next_err - err_array[std::max(0, curr_err_term - 2)]) / time_step +
               K;
    return u;
}

void PID::reset(const unit_t K, const unit_t Kp, 
                const unit_t Ki, const unit_t Kd, 
                const int err_sum_terms)
{
    // Reset tracking index to start of array
    curr_err_term = 0;
    // Reset parameters of PID
    this->K  = K;
    this->Kp = Kp,
    this->Ki = Ki,
    this->Kd = Kd;
    this->err_sum_terms = err_sum_terms;
}
