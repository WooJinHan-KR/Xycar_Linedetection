#include "LaneKeepingSystem/PIDController.hpp"
namespace Xycar {

template<typename PREC>
PIDController<PREC>::PIDController(PREC Kp_, PREC Ki_, PREC Kd_)
    : Kp(Kp_),  Ki(Ki_), Kd(Kd_), prev_error(0.0), integral(0.0), derivative(0.0) {}
    
//yemplate <typename PREC>
/*void PIDController<PREC>::setConstants(PREC Kp_, PREC Ki_, PREC Kd_){
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
}*/

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(PREC errorFromMid){
    errorFromMid;
    integral += errorFromMid;
    derivative = errorFromMid - prev_error;
    prev_error = errorFromMid;
    return Kp * errorFromMid + Ki * integral + Kd * derivative;
}



template class PIDController<float>;
template class PIDController<double>;
} // namespace Xycar