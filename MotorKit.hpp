#pragma once

#include "I2CInterface.hpp"
#include "Stepper.hpp"
#include "DCMotor.hpp"

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

  // Create stepper motor 0 or 1 using the provided motor info
  // Returns a ptr to the created stepper or nullptr if it could not be created
  Stepper* connectStepper(int id, StepperMotorInfo info);

  // Unpower and delete a stepper motor object at the specified id, 0 or 1
  void disconnectStepper(int id);

  // Create dc motor 0-3
  // Returns a ptr to the created stepper or nullptr if it could not be created
  DCMotor* connectDc(int id, StepperMotorInfo info);

  // Unpower and delete a dc motor object at the specified id: 0-3
  void disconnectDc(int id);

  // Returns a ptr to a stepper with the provided id,
  // or nullptr if none are connected
  Stepper* getStepper(int id);

  // Returns a ptr to a stepper with the provided id,
  // or nullptr if none are connected
  DCMotor* getDc(int id);

private:
  // Chip function registers
  I2CRegister<uint8_t> mode1_;
  I2CRegister<uint8_t> mode2_;
  I2CRegister<uint8_t> prescale_;
  std::vector<I2CRegister<uint16_t,uint16_t>> pwmChannels_;

  // Steppers that can be returned
  std::unique_ptr<Stepper> steppers_[2];
  std::unique_ptr<DCMotor> dcs_[4];
};
