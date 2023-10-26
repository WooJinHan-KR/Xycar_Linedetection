#include "LaneKeepingSystem/PIDController.hpp"
namespace Xycar {

template<typename PREC>
PIDController<PREC>::PIDController(PREC Kp_, PREC Ki_, PREC Kd_)
    : Kp(Kp_),  Ki(Ki_), Kd(Kd_), prev_error(0.0), integral(0.0), derivative(0.0) {}

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(PREC errorFromMid){

    errorFromMid;
    integral += errorFromMid;
    derivative = errorFromMid - prev_error;
    prev_error = errorFromMid;

    if(abs(errorFromMid) > 70)
        return 0.52 * errorFromMid + Ki * integral + Kd * derivative;

    return Kp * errorFromMid + Ki * integral + Kd * derivative;
}

template class PIDController<float>;
template class PIDController<double>;
} // namespace Xycar