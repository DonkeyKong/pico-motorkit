#pragma once

#include <chrono>

class Stepper
{
public:
  enum class Direction : int
  {
    Auto = 0,
    Forward = 1,
    Backward = -1
  };

  enum class Mode : int
  {
    Single = 0,
    Double = 1,
    Continuous = 2
  };

  virtual ~Stepper() = default;
  Stepper(const Stepper&) = delete;
  Stepper(Stepper&&) = delete;

  // Move the stepper by delta microsteps in the specified time
  // Returns false if this is not possible
  virtual bool move(int delta, std::chrono::microseconds time) = 0;

  // Move the stepper by delta microsteps at the RPM
  // Returns false if this is not possible
  virtual bool move(int delta, float rpm = 50) = 0;

  virtual bool moveDegrees(float deg, float rpm = 50)
  {
    return move(microstepsFromDegrees(deg), rpm);
  }

  // Snap the motor to the nearest whole step
  // Returns the amount snapped in microsteps
  virtual int8_t snap(Direction dir = Direction::Auto) = 0;

  // Power on last used coils, locking motor shaft in place
  virtual void grab() = 0;

  // Power off all coils, allow the motor to free spin
  virtual void release() = 0;

  // Set the motor power as a float 0-1
  // Most motors draw a LOT of current above 0.9
  // Values under 0.75 suggested
  virtual void setPower(float power) = 0;

  // Set whether the motor should single, double, or continuously step
  virtual void setMode(Mode steppingMode) = 0;

protected:
  Stepper() = default;

  // For convenience not serious use!
  virtual int microstepsFromDegrees(float deg) = 0;
};

// class Motion
// {
// public:
//   // Run this method in a loop to complete the move
//   // If called too quickly, it will block to throttle
//   // steps to the correct rate.
//   // Returns true when the move is complete
//   virtual bool stepMove() = 0

//   // Calls stepMove repeatedly until the move is complete.
//   void completeMove()
//   {
//     while (!stepMove()) {}
//   }
// protected:
//   Motion();
// };

class MotionXY
{
public:
  MotionXY(Stepper& stepperX, Stepper& stepperY, double mmPerStepX, double mmPerStepY)
    : stepperX_{stepperX}
    , stepperY_{stepperY}
    , mmPerStepX_{mmPerStepX}
    , mmPerStepY_{mmPerStepY}
  {}
  
  // Set the move destination to the specified coordinates
  // Just schedules a move. Call stepMove or completeMove 
  // to actually perform it.
  void moveTo(double x, double y, double mmPerSec)
  {

  }

  // Run this method in a loop to complete the move
  // If called too quickly, it will block to throttle
  // steps to the correct rate.
  // Returns true when the move is complete
  bool stepMove()
  {
    return false;
  }

  // Calls stepMove repeatedly until the move is complete.
  void completeMove()
  {
    
  }

private:
  Stepper& stepperX_;
  Stepper& stepperY_;
  double mmPerStepX_;
  double mmPerStepY_;
  double x_;
  double y_;
  double targetX_;
  double targetY_;
};