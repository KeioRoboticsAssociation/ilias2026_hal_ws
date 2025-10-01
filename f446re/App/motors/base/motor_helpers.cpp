/**
 * @file motor_helpers.cpp
 * @brief Helper implementations for motor controllers
 */

#include "motor_interface.hpp"
#include "../dc/dc_controller.hpp"
#include "../servo/servo_controller.hpp"
#include "../robomaster/robomaster_controller.hpp"

namespace Motors {

// Base class static method implementations
template<typename TConfig>
bool IMotorController<TConfig>::isValidPosition(float position, float minPos, float maxPos) {
    return position >= minPos && position <= maxPos;
}

template<typename TConfig>
bool IMotorController<TConfig>::isValidVelocity(float velocity, float maxVel) {
    return velocity <= maxVel;
}

template<typename TConfig>
float IMotorController<TConfig>::constrainValue(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Explicit instantiations for all config types
template class IMotorController<Config::ServoConfig>;
template class IMotorController<Config::DCMotorConfig>;
template class IMotorController<Config::RoboMasterConfig>;

// DC Controller constrainValue implementations
namespace DC {
template<typename T>
T DCMotorController::constrainValue(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

template float DCMotorController::constrainValue<float>(float, float, float);
template int DCMotorController::constrainValue<int>(int, int, int);
} // namespace DC

// Servo Controller constrainValue implementations
namespace Servo {
template<typename T>
T ServoMotorController::constrainValue(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

template float ServoMotorController::constrainValue<float>(float, float, float);
template int ServoMotorController::constrainValue<int>(int, int, int);
} // namespace Servo

// RoboMaster Controller constrainValue implementations
namespace RoboMaster {
template<typename T>
T RoboMasterMotorController::constrainValue(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

template float RoboMasterMotorController::constrainValue<float>(float, float, float);
template int RoboMasterMotorController::constrainValue<int>(int, int, int);
} // namespace RoboMaster

} // namespace Motors
