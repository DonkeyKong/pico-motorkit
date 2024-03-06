#pragma once

class DCMotor
{
public:
  virtual ~DCMotor() = default;
  DCMotor(const DCMotor&) = delete;
  DCMotor(DCMotor&&) = delete;

  // Set the motor power as a float -1 to 1
  virtual void setSpeed(float power) = 0;

protected:
  DCMotor() = default;
};