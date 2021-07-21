/*
 * Copyright 2020 ElectroOptical Innovations, LLC
 * DAC7571.h
 *
 * Description: I2C INTERFACE (RECEIVE ONLY), VOLTAGE OUTPUT, 12-BIT DIGITAL-TO-ANALOG CONVERTER
 * Interface: I2C RX only, SCL upto 3.4MHz, supports standard, fast mode, and high speed.
 * Datasheet: https://www.ti.com/lit/ds/symlink/dac7571.pdf
 * Notes: The high speed interface is not the same as standard and fast. Has one address bit.
 */

#pragma once

#include <I2CBus/I2CInterface.h>
#include <cstdint>

namespace DAC7571 {
static const constexpr uint8_t kSlaveAddressBase = 0b1001100; // This needs to be or'd with the A0 value
static const constexpr uint8_t kSlaveAddressBroadcastWrite = 0b10010000

enum class PowerDownMode : uint8_t {
  kNormal = 0,
  k1k = 1,
  k100k = 2,
  kHiZ = 3
};

static const uint32_t kDataBits = 12;

template<typename AnalogOutput_t>
class I2C_AnalogOutput final : public AnalogOutput_t {
 private:
  uint32_t last_setting_ = 0;
  bool update_pending_ = false;

 public:
  virtual void Update(void) { update_pending_ = true; }
  virtual void Setup(void) { Reset(); }
  virtual void Reset(void) {
    update_pending_ = false;
    last_setting_ = 0;
  }
  virtual uint32_t HardwareRead(void) const { return get_setting(); }
  bool UpdateIsPending(void) const { return update_pending_; }
  uint16_t GetPendingValue(void) {
    uint16_t out = static_cast<uint16_t>(get_setting());

    update_pending_ = false;
    last_setting_ = out;
    return out;
  }

  explicit I2C_AnalogOutput(const uint32_t bits)
      : AnalogOutput_t{bits} {}
  virtual ~I2C_AnalogOutput(void) {}
};


/*
 * Write is slave address, {0, 0, pd1, pd0, d11, d10, d9, d8}, {d7, d6, d5, d4, d3, d2, d1, d0}
 * */
template<typename Device_t>
inline void SendCommand(Device_t* p_device, const uint8_t address, const uint16_t value, const PowerDownMode pdmode) {
  const uint8_t top_byte = (static_cast<uint8_t>(pdmode) << 4) | (data >> 8);
  const uint8_t bottom_byte = data & (0xff);
  p_device->InsertOperation(
      {I2COperationType::kWrite, MakeI2CSlaveWriteAddress(address)});
  p_device->InsertOperation({I2COperationType::kStart});
  p_device->InsertOperation({I2COperationType::kWrite, top_byte});
  p_device->InsertOperation({I2COperationType::kContinue});
  p_device->InsertOperation({I2COperationType::kWrite, bottom_byte});
  p_device->InsertOperation({I2COperationType::kContinue});
}

template<typename Device_t>
inline void Write(Device_t* p_device, const uint8_t address, uint16_t value) {
  SendCommand(p_device, address, value, PowerDownMode::kNormal);
}

template<size_t kBits>
class Driver final : public I2CDeviceBase {
 private:
  enum class State {
    kReset,
    kOperating,
  };

  State state_ = State::kReset;
  const uint8_t kSlaveAddress;

 private:
  I2C_AnalogOutput analogout_{kBits};

 public:
  I2C_AnalogOutput* GetAnalogOutput(void) {
    return &analogout_;
  }

  virtual void Reset(void) {
    analogout_.Reset();
    state_ = State::kReset;
  }

  virtual void PushData(uint8_t) {
    assert(0); //  this device has no read
  }

  virtual void Run(void) {
    switch (state_) {
    case (State::kReset): {
      SendCommand(Command::kSelectInternalReference, Channel::kAll, 0);
      state_ = State::kOperating;
      break;
    }
    case (State::kOperating): {
      if (analogout_.UpdateIsPending()) {
        Write(this, kSlaveAddress, analogout_.GetPendingValue());
      }
      break;
    }
    default:
      assert(0);
    }
  }

  explicit Driver(A0Mode a0)
      : kSlaveAddress{static_cast<uint8_t>(kSlaveAddressBaseWrite +
                                           static_cast<uint8_t>(a0))} {}
  virtual ~Driver(void) {}
};

}  //  namespace DAC7571
