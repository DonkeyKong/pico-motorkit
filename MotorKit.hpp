#pragma once

#include "I2CInterface.hpp"
#include "Stepper.hpp"

#include <vector>
#include <chrono>
#include <memory>

class MotorKit
{
public:
  MotorKit(I2CInterface& i2c, uint8_t devAddr = 0x60, double pwmFrequency = 1600.0f);
  MotorKit(const MotorKit&) = delete;
  MotorKit(MotorKit&&) = delete;
  ~MotorKit() = default;

  Stepper& getStepper(int id);
  Stepper& stepper0();
  Stepper& stepper1();

private:
  // Chip function registers
  I2CRegister<uint8_t> mode1_;
  I2CRegister<uint8_t> mode2_;
  I2CRegister<uint8_t> prescale_;
  std::vector<I2CRegister<uint16_t,uint16_t>> pwmChannels_;

  // Steppers that can be returned
  std::unique_ptr<Stepper> stepper0_;
  std::unique_ptr<Stepper> stepper1_;
};
