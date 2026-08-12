#include "serial_arm_protocol_hiwonder_bus_servo/bus.hpp"

#include <gtest/gtest.h>

#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace {

namespace hiwonder = serial_arm::protocol::hiwonder_bus_servo;
using hiwonder::Buffer;
using hiwonder::Err;

class Pty {
public:
    Pty() {
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        if(master_ < 0) throw std::runtime_error("posix_openpt failed");
        if(::grantpt(master_) != 0 || ::unlockpt(master_) != 0) {
            throw std::runtime_error("pty setup failed");
        }
        const char* name = ::ptsname(master_);
        if(name == nullptr) throw std::runtime_error("ptsname failed");
        slave_ = name;
    }

    ~Pty() {
        if(master_ >= 0) (void)::close(master_);
    }

    int master() const noexcept { return master_; }
    const std::string& slave() const noexcept { return slave_; }

private:
    int master_{ -1 };
    std::string slave_;
};

serial_arm::transport::SerialPort open_serial(const Pty& pty) {
    serial_arm::transport::SerialPort::Config config;
    config.baud_rate = 115200;
    config.read_timeout = std::chrono::milliseconds(20);
    config.write_timeout = std::chrono::milliseconds(20);
    return serial_arm::transport::SerialPort(pty.slave(), config);
}

Buffer status_packet(std::uint8_t id, std::uint8_t error, const Buffer& parameters = {}) {
    Buffer packet{ 0xFF, 0xFF, id, static_cast<std::uint8_t>(parameters.size() + 2U), error };
    packet.insert(packet.end(), parameters.begin(), parameters.end());
    packet.push_back(hiwonder::checksum(packet));
    return packet;
}

bool read_exact_fd(int fd, Buffer& buffer, std::size_t size) {
    buffer.resize(size);
    std::size_t offset = 0;
    while(offset < size) {
        const ssize_t n = ::read(fd, buffer.data() + offset, size - offset);
        if(n <= 0) return false;
        offset += static_cast<std::size_t>(n);
    }
    return true;
}

} // namespace

TEST(HiwonderPacketTests, CalculatesDocumentedChecksum) {
    const Buffer packet{ 0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02 };
    EXPECT_EQ(hiwonder::checksum(packet), 0xBE);
}

TEST(HiwonderPacketTests, EncodesReadPacket) {
    const auto packet = hiwonder::encode_read_packet(1U, hiwonder::PRESENT_POSITION_ADDR, 2U);
    ASSERT_TRUE(packet);
    EXPECT_EQ(*packet, (Buffer{ 0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE }));
}

TEST(HiwonderPacketTests, EncodesWritePacket) {
    const auto packet = hiwonder::encode_write_packet(
        1U,
        0x2A,
        { 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03 });
    ASSERT_TRUE(packet);
    EXPECT_EQ(*packet, (Buffer{
        0xFF, 0xFF, 0x01, 0x09, 0x03, 0x2A,
        0x00, 0x08, 0x00, 0x00, 0xE8, 0x03, 0xD5,
        }));
}

TEST(HiwonderPacketTests, ParsesWriteStatusAck) {
    const auto status = hiwonder::parse_status_packet({ 0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC });
    ASSERT_TRUE(status);
    EXPECT_EQ(status->id, 1U);
    EXPECT_EQ(status->error, 0U);
    EXPECT_TRUE(status->parameters.empty());
}

TEST(HiwonderPacketTests, EncodesSyncReadPacket) {
    const auto packet = hiwonder::encode_sync_read_packet({ 1U, 2U }, 0x38, 8U);
    ASSERT_TRUE(packet);
    EXPECT_EQ(*packet, (Buffer{ 0xFF, 0xFF, 0xFE, 0x06, 0x82, 0x38, 0x08, 0x01, 0x02, 0x36 }));
}

TEST(HiwonderPacketTests, EncodesSyncWritePwmPacket) {
    const auto packet = hiwonder::encode_sync_write_packet(
        hiwonder::PWM_COMMAND_ADDR,
        2U,
        {
            hiwonder::SyncWriteEntry{ 1U, { 0x64, 0x00 } },
            hiwonder::SyncWriteEntry{ 2U, { 0xC8, 0x04 } },
        });
    ASSERT_TRUE(packet);
    EXPECT_EQ(*packet, (Buffer{
        0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x2C, 0x02,
        0x01, 0x64, 0x00,
        0x02, 0xC8, 0x04,
        0x13,
        }));
}

TEST(HiwonderPacketTests, DecodesRawStateFields) {
    const hiwonder::StatusPacket state_packet{
        1U,
        0U,
        {
            0x18, 0x05,
            0xE8, 0x03,
            0x2C, 0x05,
            0x79,
            0x1E,
            0x00,
            0x21,
        },
    };
    const hiwonder::StatusPacket current_packet{ 1U, 0U, { 0x34, 0x12 } };

    const auto state = hiwonder::decode_raw_state(state_packet, current_packet);
    ASSERT_TRUE(state);
    EXPECT_EQ(state->position_raw, 1304);
    EXPECT_EQ(state->velocity_raw, 1000U);
    EXPECT_EQ(state->load_raw, -300);
    EXPECT_EQ(state->voltage_raw, 0x79U);
    EXPECT_EQ(state->temperature_raw, 0x1EU);
    EXPECT_EQ(state->fault, 0x21U);
    EXPECT_EQ(state->current_raw_ma, 0x1234U);
}

TEST(HiwonderPacketTests, DecodesStateBlockWithoutCurrentTransaction) {
    const hiwonder::StatusPacket state_packet{
        2U,
        0U,
        {
            0x00, 0x08,
            0x10, 0x00,
            0x00, 0x00,
            0x7B,
            0x24,
            0x00,
            0x00,
        },
    };

    const auto state = hiwonder::decode_state_block(state_packet, 321U);
    ASSERT_TRUE(state);
    EXPECT_EQ(state->id, 2U);
    EXPECT_EQ(state->position_raw, 2048);
    EXPECT_EQ(state->velocity_raw, 16U);
    EXPECT_EQ(state->current_raw_ma, 321U);
}

TEST(HiwonderPacketTests, DecodesSignedAbsolutePosition) {
    const hiwonder::StatusPacket state_packet{
        1U,
        0U,
        {
            0x01, 0x80,
            0x00, 0x00,
            0x00, 0x00,
            0x78,
            0x20,
            0x00,
            0x00,
        },
    };
    const hiwonder::StatusPacket current_packet{ 1U, 0U, { 0x00, 0x00 } };

    const auto state = hiwonder::decode_raw_state(state_packet, current_packet);
    ASSERT_TRUE(state);
    EXPECT_EQ(state->position_raw, -1);
}

TEST(HiwonderBusServoTests, ReadsPositionCalibrationWithBit11Sign) {
    Pty pty;
    auto serial = open_serial(pty);
    hiwonder::HiwonderBusServo bus(serial);
    bool responder_ok = true;

    std::thread responder([&]() {
        Buffer request;
        responder_ok = read_exact_fd(pty.master(), request, 8U);
        if(!responder_ok) return;
        responder_ok = request[5] == hiwonder::POSITION_CALIBRATION_ADDR && request[6] == 2U;
        if(!responder_ok) return;
        const auto positive = status_packet(4U, 0U, { 0x21, 0x07 });
        responder_ok = ::write(pty.master(), positive.data(), positive.size()) ==
            static_cast<ssize_t>(positive.size());
        });

    const auto calibration = bus.read_position_calibration(4U, std::chrono::milliseconds(20));
    responder.join();
    ASSERT_TRUE(responder_ok);
    ASSERT_TRUE(calibration);
    EXPECT_EQ(*calibration, 1825);
}

TEST(HiwonderBusServoTests, ReadsNegativePositionCalibrationWithBit11Sign) {
    Pty pty;
    auto serial = open_serial(pty);
    hiwonder::HiwonderBusServo bus(serial);

    std::thread responder([&]() {
        Buffer request;
        ASSERT_TRUE(read_exact_fd(pty.master(), request, 8U));
        const auto negative = status_packet(4U, 0U, { 0x21, 0x0F });
        ASSERT_EQ(::write(pty.master(), negative.data(), negative.size()),
            static_cast<ssize_t>(negative.size()));
        });

    const auto calibration = bus.read_position_calibration(4U, std::chrono::milliseconds(20));
    responder.join();
    ASSERT_TRUE(calibration);
    EXPECT_EQ(*calibration, -1825);
}

TEST(HiwonderPacketTests, RejectsMalformedPackets) {
    auto bad_header = status_packet(1U, 0U);
    bad_header[0] = 0x00;
    auto bad_length = status_packet(1U, 0U, { 0x01 });
    bad_length[3] = 0x02;
    auto bad_checksum = status_packet(1U, 0U);
    bad_checksum.back() ^= 0x01;

    const auto header_result = hiwonder::parse_status_packet(bad_header);
    const auto length_result = hiwonder::parse_status_packet(bad_length);
    const auto checksum_result = hiwonder::parse_status_packet(bad_checksum);
    ASSERT_FALSE(header_result);
    ASSERT_FALSE(length_result);
    ASSERT_FALSE(checksum_result);
    EXPECT_EQ(header_result.error(), Err::MALFORMED_PACKET);
    EXPECT_EQ(length_result.error(), Err::MALFORMED_PACKET);
    EXPECT_EQ(checksum_result.error(), Err::CHECKSUM_MISMATCH);
}

TEST(HiwonderPacketTests, RejectsInvalidIdsAndDuplicateSyncIds) {
    const auto broadcast_read = hiwonder::encode_read_packet(hiwonder::BROADCAST_ID, 0x38, 2U);
    const auto duplicate_sync_read = hiwonder::encode_sync_read_packet({ 1U, 1U }, 0x38, 2U);
    ASSERT_FALSE(broadcast_read);
    ASSERT_FALSE(duplicate_sync_read);
    EXPECT_EQ(broadcast_read.error(), Err::INVALID_ARGUMENT);
    EXPECT_EQ(duplicate_sync_read.error(), Err::INVALID_ARGUMENT);
}

TEST(HiwonderBusServoTests, ConsumesWriteAckBeforeFollowingRead) {
    Pty pty;
    auto serial = open_serial(pty);
    hiwonder::HiwonderBusServo bus(serial);
    bool responder_ok = true;

    std::thread responder([&]() {
        Buffer request;
        responder_ok = read_exact_fd(pty.master(), request, 8U);
        if(!responder_ok) return;
        const auto ack = status_packet(1U, 0U);
        responder_ok = ::write(pty.master(), ack.data(), ack.size()) == static_cast<ssize_t>(ack.size());
        if(!responder_ok) return;

        responder_ok = read_exact_fd(pty.master(), request, 8U);
        if(!responder_ok) return;
        const auto position = status_packet(1U, 0U, { 0x18, 0x05 });
        responder_ok = ::write(pty.master(), position.data(), position.size()) ==
            static_cast<ssize_t>(position.size());
        });

    const auto write = bus.set_torque_enable(1U, true, std::chrono::milliseconds(20));
    const auto position = bus.read_position(1U, std::chrono::milliseconds(20));
    responder.join();

    ASSERT_TRUE(responder_ok);
    ASSERT_TRUE(write);
    ASSERT_TRUE(position);
    EXPECT_EQ(*position, 1304U);
}

TEST(HiwonderBusServoTests, SyncWritesSignedPwmInOnePacket) {
    Pty pty;
    auto serial = open_serial(pty);
    hiwonder::HiwonderBusServo bus(serial);

    const auto result = bus.sync_write_pwm({
        hiwonder::PwmCommand{ 1U, 100 },
        hiwonder::PwmCommand{ 2U, -200 },
        });
    ASSERT_TRUE(result);

    Buffer packet;
    ASSERT_TRUE(read_exact_fd(pty.master(), packet, 14U));
    EXPECT_EQ(packet, (Buffer{
        0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x2C, 0x02,
        0x01, 0x64, 0x00,
        0x02, 0xC8, 0x04,
        0x13,
        }));
}

TEST(HiwonderBusServoTests, ReportsDeviceErrorFromWriteAck) {
    Pty pty;
    auto serial = open_serial(pty);
    hiwonder::HiwonderBusServo bus(serial);

    std::thread responder([&]() {
        Buffer request;
        ASSERT_TRUE(read_exact_fd(pty.master(), request, 8U));
        const auto ack = status_packet(1U, 0x04);
        ASSERT_EQ(::write(pty.master(), ack.data(), ack.size()), static_cast<ssize_t>(ack.size()));
        });

    const auto result = bus.set_torque_enable(1U, true, std::chrono::milliseconds(20));
    responder.join();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), Err::DEVICE_ERROR);
}

TEST(HiwonderBusServoTests, TimesOutWhenReadHasNoResponse) {
    Pty pty;
    auto serial = open_serial(pty);
    hiwonder::HiwonderBusServo bus(serial);

    const auto result = bus.read_position(1U, std::chrono::milliseconds(10));
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), Err::TIMEOUT);
}
