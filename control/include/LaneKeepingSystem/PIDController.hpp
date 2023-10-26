#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <yaml-cpp/yaml.h>


namespace Xycar {

/**
 * @brief PID Controller Class
 * @tparam PREC Precision of data
 */


template <typename PREC>
class PIDController final
{
public:
    using Ptr = std::unique_ptr<PIDController>;

    PIDController(PREC kp_, PREC Ki_, PREC Kd_);

    PREC getControlOutput(PREC errorFromMid);

private:
    const PREC Kp, Ki, Kd;
    PREC integral;
    PREC prev_error;
    PREC derivative;
};
} // namespace Xycar
#endif // PID_CONTROLLER_HPP_
