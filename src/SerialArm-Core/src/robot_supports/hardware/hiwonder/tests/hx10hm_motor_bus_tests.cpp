#include "serial_arm_hardware_hiwonder/hx10hm_motor_bus.hpp"

#include "serial_arm/hardware/hardware_loader.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace {

constexpr double TWO_PI = 6.28318530717958647692;
constexpr double RAD_PER_STEP = TWO_PI / 4096.0;

std::string fixture(const std::string& name) {
    return std::string(SERIAL_ARM_HIWONDER_TEST_FIXTURE_DIR) + "/" + name;
}

serial_arm::HiwonderBusCfg valid_cfg() {
    serial_arm::HiwonderBusCfg cfg;
    cfg.serial_port = "/dev/null";
    cfg.baudrate = 1000000;
    cfg.read_timeout = std::chrono::milliseconds(5);
    cfg.write_timeout = std::chrono::milliseconds(20);
    cfg.feedback_timeout = std::chrono::milliseconds(50);
    cfg.startup_read_cycles = 2;
    cfg.restore_position_mode_on_deactivate = false;
    cfg.velocity_encoding = serial_arm::HiwonderVelocityEncoding::Bit15SignMagnitude;
    cfg.torque_feedback_mode = "unavailable_zero";
    for(std::size_t i = 0; i < 6U; ++i) {
        serial_arm::HiwonderActuatorCfg actuator;
        actuator.name = "test_motor_" + std::to_string(i + 1U);
        actuator.joint_name = "test_joint_" + std::to_string(i + 1U);
        actuator.servo_id = static_cast<std::uint8_t>(i + 1U);
        actuator.raw_zero = 2048;
        actuator.direction = i % 2U == 0U ? 1 : -1;
        actuator.min_pos = -2.0;
        actuator.max_pos = 2.0;
        actuator.max_vel = 3.0;
        actuator.max_effort = 2.0;
        actuator.max_kp = 10.0;
        actuator.max_kd = 5.0;
        actuator.positive_gain = 100.0 + static_cast<double>(i);
        actuator.negative_gain = 120.0 + static_cast<double>(i);
        actuator.positive_offset = 10.0 + static_cast<double>(i);
        actuator.negative_offset = 20.0 + static_cast<double>(i);
        actuator.torque_deadband_nm = 0.05;
        actuator.pwm_limit = 500;
        cfg.actuators.push_back(std::move(actuator));
    }
    return cfg;
}

serial_arm::ActuatorCtrlCmd valid_cmd(const serial_arm::HiwonderBusCfg& cfg) {
    serial_arm::ActuatorCtrlCmd cmd;
    const std::size_t n = cfg.actuators.size();
    cmd.pos.assign(n, 0.0);
    cmd.vel.assign(n, 0.0);
    cmd.tor.assign(n, 0.0);
    cmd.kp.assign(n, 0.0);
    cmd.kd.assign(n, 0.0);
    return cmd;
}

serial_arm::ActuatorState valid_state(const serial_arm::HiwonderBusCfg& cfg) {
    serial_arm::ActuatorState state;
    const std::size_t n = cfg.actuators.size();
    state.pos.assign(n, 0.0);
    state.vel.assign(n, 0.0);
    state.tor.assign(n, 0.0);
    state.online.assign(n, 1U);
    state.enabled.assign(n, 1U);
    state.err_code.assign(n, 0);
    return state;
}

std::int16_t first_pwm(
    serial_arm::Hx10hmMotorBus& bus,
    const serial_arm::ActuatorCtrlCmd& cmd,
    const serial_arm::ActuatorState& state) {
    const auto commands = bus.build_pwm_commands(cmd, state, std::chrono::milliseconds(0));
    EXPECT_TRUE(commands);
    if(!commands || commands->empty()) return 0;
    return (*commands)[0].pwm;
}

} // namespace

TEST(Hx10hmMotorBusConfigTests, LoadsExplicitBackendMappingTestFixture) {
    serial_arm::Hx10hmMotorBus bus;
    const auto configured = bus.configure(fixture("backend_mapping_test_fixture.yaml"));
    ASSERT_TRUE(configured);
    EXPECT_EQ(bus.size(), 6U);
    ASSERT_EQ(bus.capabilities().size(), 6U);
    EXPECT_EQ(bus.capabilities()[0].actuator_name, "test_motor_1");
    EXPECT_DOUBLE_EQ(bus.capabilities()[0].max_effort, 2.0);
}

TEST(Hx10hmMotorBusConfigTests, LoadsOfficialDerivedNominalConfiguration) {
    serial_arm::Hx10hmMotorBus bus;
    const auto configured = bus.configure(SERIAL_ARM_HIWONDER_NOMINAL_CONFIG);
    ASSERT_TRUE(configured);
    ASSERT_EQ(bus.capabilities().size(), 6U);
    for(const auto& capability : bus.capabilities()) {
        EXPECT_NEAR(capability.min_pos, -3.141592654, 1e-12);
        EXPECT_NEAR(capability.max_pos, 3.140058673, 1e-12);
        EXPECT_NEAR(capability.max_vel, 10.471976, 1e-12);
        EXPECT_NEAR(capability.max_effort, 0.2941995, 1e-12);
        EXPECT_NEAR(capability.max_kp, 1.0, 1e-12);
        EXPECT_NEAR(capability.max_kd, 0.1, 1e-12);
    }
    const auto full_nominal = bus.torque_to_pwm(0U, 0.2941995);
    ASSERT_TRUE(full_nominal);
    EXPECT_EQ(*full_nominal, 300);

    const YAML::Node actuators = YAML::LoadFile(
        SERIAL_ARM_HIWONDER_NOMINAL_CONFIG)["hiwonder"]["actuators"];
    ASSERT_TRUE(actuators.IsSequence());
    ASSERT_EQ(actuators.size(), 6U);
    for(std::size_t i = 0; i < actuators.size(); ++i) {
        EXPECT_EQ(actuators[i]["servo_id"].as<int>(), static_cast<int>(i + 1U));
        EXPECT_EQ(actuators[i]["raw_zero"].as<int>(), 2048);
        EXPECT_EQ(actuators[i]["direction"].as<int>(), 1);
        EXPECT_DOUBLE_EQ(actuators[i]["positive_gain"].as<double>(), 1019.716213);
        EXPECT_DOUBLE_EQ(actuators[i]["negative_gain"].as<double>(), 1019.716213);
        EXPECT_DOUBLE_EQ(actuators[i]["positive_offset"].as<double>(), 0.0);
        EXPECT_DOUBLE_EQ(actuators[i]["negative_offset"].as<double>(), 0.0);
        EXPECT_DOUBLE_EQ(actuators[i]["torque_deadband_nm"].as<double>(), 0.0);
        EXPECT_EQ(actuators[i]["pwm_limit"].as<int>(), 300);
    }
}

TEST(Hx10hmMotorBusMappingTests, ProducesExpectedNominalSmallTorquePwm) {
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(SERIAL_ARM_HIWONDER_NOMINAL_CONFIG));
    const auto pwm_002 = bus.torque_to_pwm(0U, 0.02);
    const auto pwm_005 = bus.torque_to_pwm(0U, 0.05);
    const auto pwm_010 = bus.torque_to_pwm(0U, 0.10);
    const auto pwm_negative = bus.torque_to_pwm(0U, -0.10);
    ASSERT_TRUE(pwm_002);
    ASSERT_TRUE(pwm_005);
    ASSERT_TRUE(pwm_010);
    ASSERT_TRUE(pwm_negative);
    EXPECT_EQ(*pwm_002, 20);
    EXPECT_EQ(*pwm_005, 51);
    EXPECT_EQ(*pwm_010, 102);
    EXPECT_EQ(*pwm_negative, -102);
}

TEST(Hx10hmMotorBusConfigTests, RejectsInvalidActuatorCount) {
    auto cfg = valid_cfg();
    cfg.actuators.pop_back();
    serial_arm::Hx10hmMotorBus bus;
    const auto configured = bus.configure(cfg);
    ASSERT_FALSE(configured);
    EXPECT_EQ(configured.error(), serial_arm::MotorBusErr::INVALID_CFG);
}

TEST(Hx10hmMotorBusConfigTests, RejectsDuplicatedServoId) {
    auto cfg = valid_cfg();
    cfg.actuators[5].servo_id = cfg.actuators[0].servo_id;
    serial_arm::Hx10hmMotorBus bus;
    const auto configured = bus.configure(cfg);
    ASSERT_FALSE(configured);
    EXPECT_EQ(configured.error(), serial_arm::MotorBusErr::INVALID_CFG);
}

TEST(Hx10hmMotorBusConfigTests, RejectsNonFiniteCalibration) {
    auto cfg = valid_cfg();
    cfg.actuators[2].positive_gain = std::numeric_limits<double>::quiet_NaN();
    serial_arm::Hx10hmMotorBus bus;
    const auto configured = bus.configure(cfg);
    ASSERT_FALSE(configured);
    EXPECT_EQ(configured.error(), serial_arm::MotorBusErr::INVALID_CFG);
}

TEST(Hx10hmMotorBusConversionTests, AppliesRawZeroAndDirectionToPosition) {
    EXPECT_NEAR(serial_arm::Hx10hmMotorBus::raw_position_to_rad(2148, 2048U, 1),
        100.0 * RAD_PER_STEP, 1e-12);
    EXPECT_NEAR(serial_arm::Hx10hmMotorBus::raw_position_to_rad(2148, 2048U, -1),
        -100.0 * RAD_PER_STEP, 1e-12);
}

TEST(Hx10hmMotorBusConversionTests, AcceptsSignedAbsolutePositionFeedback) {
    EXPECT_NEAR(serial_arm::Hx10hmMotorBus::raw_position_to_rad(4096, 2048U, 1),
        2048.0 * RAD_PER_STEP, 1e-12);
    EXPECT_NEAR(serial_arm::Hx10hmMotorBus::raw_position_to_rad(-1, 2048U, 1),
        -2049.0 * RAD_PER_STEP, 1e-12);
}

TEST(Hx10hmMotorBusConversionTests, DecodesVelocityBit15AndDirection) {
    EXPECT_NEAR(serial_arm::Hx10hmMotorBus::raw_velocity_to_rad_per_second(100U, 1),
        100.0 * RAD_PER_STEP, 1e-12);
    EXPECT_NEAR(serial_arm::Hx10hmMotorBus::raw_velocity_to_rad_per_second(0x8064U, -1),
        100.0 * RAD_PER_STEP, 1e-12);
}

TEST(Hx10hmMotorBusMitTests, ComputesTauOnlyCommand) {
    auto cfg = valid_cfg();
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    auto cmd = valid_cmd(cfg);
    const auto state = valid_state(cfg);
    cmd.tor[0] = 0.5;
    EXPECT_EQ(first_pwm(bus, cmd, state), 60);
}

TEST(Hx10hmMotorBusMitTests, ComputesKpOnlyVirtualSpring) {
    auto cfg = valid_cfg();
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    auto cmd = valid_cmd(cfg);
    const auto state = valid_state(cfg);
    cmd.pos[0] = 0.5;
    cmd.kp[0] = 2.0;
    EXPECT_EQ(first_pwm(bus, cmd, state), 110);
}

TEST(Hx10hmMotorBusMitTests, ComputesKdOnlyDamping) {
    auto cfg = valid_cfg();
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    auto cmd = valid_cmd(cfg);
    auto state = valid_state(cfg);
    cmd.vel[0] = 0.2;
    cmd.kd[0] = 1.0;
    state.vel[0] = 0.5;
    EXPECT_EQ(first_pwm(bus, cmd, state), -56);
}

TEST(Hx10hmMotorBusMitTests, AddsAllThreeMitTerms) {
    auto cfg = valid_cfg();
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    auto cmd = valid_cmd(cfg);
    auto state = valid_state(cfg);
    cmd.tor[0] = 0.2;
    cmd.pos[0] = 0.5;
    cmd.kp[0] = 2.0;
    cmd.vel[0] = 0.2;
    cmd.kd[0] = 1.0;
    state.pos[0] = 0.1;
    state.vel[0] = 0.3;
    EXPECT_EQ(first_pwm(bus, cmd, state), 100);
}

TEST(Hx10hmMotorBusMappingTests, MapsPositiveAndNegativeTorque) {
    const auto cfg = valid_cfg();
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    const auto positive = bus.torque_to_pwm(0U, 0.5);
    const auto negative = bus.torque_to_pwm(0U, -0.5);
    ASSERT_TRUE(positive);
    ASSERT_TRUE(negative);
    EXPECT_EQ(*positive, 60);
    EXPECT_EQ(*negative, -80);
}

TEST(Hx10hmMotorBusMappingTests, AppliesDirectionToRawMotorTorqueMapping) {
    auto cfg = valid_cfg();
    cfg.actuators[0].direction = 1;
    cfg.actuators[1].direction = -1;
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));

    const auto direction_positive_tau_positive = bus.torque_to_pwm(0U, 0.1);
    const auto direction_positive_tau_negative = bus.torque_to_pwm(0U, -0.1);
    const auto direction_negative_tau_positive = bus.torque_to_pwm(1U, 0.1);
    const auto direction_negative_tau_negative = bus.torque_to_pwm(1U, -0.1);
    ASSERT_TRUE(direction_positive_tau_positive);
    ASSERT_TRUE(direction_positive_tau_negative);
    ASSERT_TRUE(direction_negative_tau_positive);
    ASSERT_TRUE(direction_negative_tau_negative);
    EXPECT_EQ(*direction_positive_tau_positive, 20);
    EXPECT_EQ(*direction_positive_tau_negative, -32);
    EXPECT_EQ(*direction_negative_tau_positive, -33);
    EXPECT_EQ(*direction_negative_tau_negative, 21);
}

TEST(Hx10hmMotorBusMappingTests, AppliesDirectionNegativeDeadbandAndSaturation) {
    auto cfg = valid_cfg();
    cfg.actuators[1].direction = -1;
    cfg.actuators[1].negative_gain = 400.0;
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));

    const auto deadband_positive = bus.torque_to_pwm(1U, 0.05);
    const auto deadband_negative = bus.torque_to_pwm(1U, -0.05);
    const auto pwm_saturated = bus.torque_to_pwm(1U, 2.0);
    const auto effort_saturated = bus.torque_to_pwm(1U, 20.0);
    ASSERT_TRUE(deadband_positive);
    ASSERT_TRUE(deadband_negative);
    ASSERT_TRUE(pwm_saturated);
    ASSERT_TRUE(effort_saturated);
    EXPECT_EQ(*deadband_positive, 0);
    EXPECT_EQ(*deadband_negative, 0);
    EXPECT_EQ(*pwm_saturated, -500);
    EXPECT_EQ(*effort_saturated, *pwm_saturated);
}

TEST(Hx10hmMotorBusMappingTests, AppliesDeadbandBeforeOffsets) {
    const auto cfg = valid_cfg();
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    EXPECT_EQ(*bus.torque_to_pwm(0U, 0.05), 0);
    EXPECT_EQ(*bus.torque_to_pwm(0U, -0.05), 0);
}

TEST(Hx10hmMotorBusMappingTests, SaturatesPwm) {
    auto cfg = valid_cfg();
    cfg.actuators[0].positive_gain = 400.0;
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    const auto pwm = bus.torque_to_pwm(0U, 2.0);
    ASSERT_TRUE(pwm);
    EXPECT_EQ(*pwm, 500);
}

TEST(Hx10hmMotorBusMappingTests, SaturatesEffortBeforeMapping) {
    const auto cfg = valid_cfg();
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    const auto at_limit = bus.torque_to_pwm(0U, 2.0);
    const auto over_limit = bus.torque_to_pwm(0U, 20.0);
    ASSERT_TRUE(at_limit);
    ASSERT_TRUE(over_limit);
    EXPECT_EQ(*at_limit, 210);
    EXPECT_EQ(*over_limit, *at_limit);
}

TEST(Hx10hmMotorBusSafetyTests, RejectsStaleState) {
    const auto cfg = valid_cfg();
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    const auto commands = bus.build_pwm_commands(
        valid_cmd(cfg),
        valid_state(cfg),
        cfg.feedback_timeout + std::chrono::milliseconds(1));
    ASSERT_FALSE(commands);
    EXPECT_EQ(commands.error(), serial_arm::MotorBusErr::TIMEOUT);
}

TEST(Hx10hmMotorBusSafetyTests, RejectsNonFiniteCommand) {
    const auto cfg = valid_cfg();
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    auto cmd = valid_cmd(cfg);
    cmd.tor[0] = std::numeric_limits<double>::infinity();
    const auto commands = bus.build_pwm_commands(
        cmd,
        valid_state(cfg),
        std::chrono::milliseconds(0));
    ASSERT_FALSE(commands);
    EXPECT_EQ(commands.error(), serial_arm::MotorBusErr::INVALID_CMD);
}

TEST(Hx10hmMotorBusSafetyTests, PreservesSixAxisServoIdOrder) {
    auto cfg = valid_cfg();
    const std::vector<std::uint8_t> ids{ 6U, 4U, 2U, 5U, 3U, 1U };
    for(std::size_t i = 0; i < ids.size(); ++i) cfg.actuators[i].servo_id = ids[i];
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    auto cmd = valid_cmd(cfg);
    cmd.tor.assign(ids.size(), 0.1);
    const auto commands = bus.build_pwm_commands(
        cmd,
        valid_state(cfg),
        std::chrono::milliseconds(0));
    ASSERT_TRUE(commands);
    ASSERT_EQ(commands->size(), ids.size());
    for(std::size_t i = 0; i < ids.size(); ++i) EXPECT_EQ((*commands)[i].id, ids[i]);
}

TEST(Hx10hmMotorBusCommandTests, RejectsWrongDimensionsAndCapabilityViolation) {
    const auto cfg = valid_cfg();
    serial_arm::Hx10hmMotorBus bus;
    ASSERT_TRUE(bus.configure(cfg));
    auto wrong_size = valid_cmd(cfg);
    wrong_size.pos.pop_back();
    auto over_limit = valid_cmd(cfg);
    over_limit.kp[0] = cfg.actuators[0].max_kp + 0.1;
    EXPECT_FALSE(bus.validate_command(wrong_size));
    EXPECT_FALSE(bus.validate_command(over_limit));
}

TEST(Hx10hmMotorBusLifecycleTests, RejectsOperationsInInvalidStates) {
    serial_arm::Hx10hmMotorBus bus;
    const auto connect = bus.connect();
    const auto read = bus.read();
    ASSERT_FALSE(connect);
    ASSERT_FALSE(read);
    EXPECT_EQ(connect.error(), serial_arm::MotorBusErr::NOT_CONFIGURED);
    EXPECT_EQ(read.error(), serial_arm::MotorBusErr::NOT_CONNECTED);

    const auto cfg = valid_cfg();
    ASSERT_TRUE(bus.configure(cfg));
    const auto write = bus.write(valid_cmd(cfg));
    const auto stop = bus.stop();
    ASSERT_FALSE(write);
    ASSERT_FALSE(stop);
    EXPECT_EQ(write.error(), serial_arm::MotorBusErr::NOT_ACTIVE);
    EXPECT_EQ(stop.error(), serial_arm::MotorBusErr::NOT_ACTIVE);
    EXPECT_NO_THROW(bus.cleanup());
    EXPECT_NO_THROW(bus.cleanup());
}

TEST(Hx10hmMotorBusPluginTests, LoadsThroughHardwareLoaderAndKeepsAbiEntry) {
    serial_arm::HardwareLoader loader;
    const auto loaded = loader.load(
        SERIAL_ARM_HIWONDER_TEST_PLUGIN,
        fixture("backend_mapping_test_fixture.yaml"));
    ASSERT_TRUE(loaded);
    ASSERT_TRUE(*loaded);
    EXPECT_EQ((*loaded)->size(), 6U);
}
