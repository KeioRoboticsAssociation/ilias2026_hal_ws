/**
 * @file motor_stubs.cpp
 * @brief Stub implementations for motor controller callbacks
 */

#include "dc/dc_controller.hpp"
#include "servo/servo_controller.hpp"
#include "robomaster/robomaster_controller.hpp"

namespace Motors {
namespace DC {

void DCMotorController::setErrorCallback(std::function<void(uint8_t, Config::ErrorCode)> callback) {
    (void)callback;
}

void DCMotorController::setStateCallback(std::function<void(uint8_t, const BaseMotorState&)> callback) {
    (void)callback;
}

} // namespace DC

namespace Servo {

void ServoMotorController::setErrorCallback(std::function<void(uint8_t, Config::ErrorCode)> callback) {
    (void)callback;
}

void ServoMotorController::setStateCallback(std::function<void(uint8_t, const BaseMotorState&)> callback) {
    (void)callback;
}

} // namespace Servo

namespace RoboMaster {

void RoboMasterMotorController::setErrorCallback(std::function<void(uint8_t, Config::ErrorCode)> callback) {
    (void)callback;
}

void RoboMasterMotorController::setStateCallback(std::function<void(uint8_t, const BaseMotorState&)> callback) {
    (void)callback;
}

} // namespace RoboMaster
} // namespace Motors
