#include "serial_arm_hardware_hiwonder/hx10hm_motor_bus.hpp"

extern "C" serial_arm::MotorBus* create_motor_bus() {
    return new serial_arm::Hx10hmMotorBus();
}

extern "C" void destroy_motor_bus(serial_arm::MotorBus* bus) {
    delete bus;
}
