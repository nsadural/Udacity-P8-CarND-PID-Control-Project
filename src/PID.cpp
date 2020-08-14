#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    step = 1;
    gain_index = 0;
    select_case = 0;
    current_error = 0.0;
    total_error = 0.0;
    best_error = std::numeric_limits<double>::max();
    K.push_back(Kp);
    K.push_back(Ki);
    K.push_back(Kd);
    dK.push_back(0.05 * Kp);
    dK.push_back(0.05 * Ki);
    dK.push_back(0.05 * Kd);

}

void PID::UpdateError(double cte) {

    if (!is_initialized) {
        p_error = cte;
        is_initialized = true;
    }

    d_error = cte - p_error;
    i_error += cte;
    p_error = cte;

    /***** Parameter Optimization *****/

    current_error = cte * cte;
    total_error += current_error;

    if ((step % update_period) == 0) {

        switch (select_case) {

        case 0:

            K[gain_index] += dK[gain_index];

            select_case++;

            break;

        case 1:

            if (total_error < best_error) {
                best_error = total_error;
                dK[gain_index] *= 1.1;
            }
            else {
                K[gain_index] -= 2 * dK[gain_index];
            }

            select_case++;

            break;

        case 2:

            if (total_error < best_error) {
                best_error = total_error;
                dK[gain_index] *= 1.1;
            }
            else {
                K[gain_index] += dK[gain_index];
                dK[gain_index] *= 0.9;
            }

            select_case = 0;

            gain_index++;
            if (gain_index > 2) {
                gain_index = 0;
            }

            break;

        }

        total_error = 0.0;
        if (gain_index == 0) {
            Kp = K[gain_index];
        }
        else if (gain_index == 1) {
            Ki = K[gain_index];
        }
        else {
            Kd = K[gain_index];
        }

        // DEBUG
        //std::cout << "Kp: " << Kp << "\tKi: " << Ki << "\tKd: " << Kd << "\tbest_error: " << best_error << std::endl;

    }

    step++;

}

double PID::UpdateControl() {
  
  return -Kp*p_error - Ki*i_error - Kd*d_error; 
}