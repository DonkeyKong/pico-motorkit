#pragma once

#include <chrono>

// Information about the stepper motors being connected
struct StepperMotorInfo
{
  static const uint64_t MicrosecondsPerMinute = (1000000 * 60);
  static const int StepsPerCycle = 4; // This is true of all supported steppers
  static const int DegPerRev = 360.0f;

StepperMotorInfo()
    : Valid {false}
    , StepsPerRev {0}
    , MicrostepsPerStep {0}
    , MicrostepsPerCycle {0}
    , StepsPerDegree {0}
    , MicrostepsPerRev {0}
    , MicrostepsPerDegree {0}
    , MaxRpm {0}
    , MinimumUsPerStep {0}
    , ReverseA {false}
  { }

  StepperMotorInfo(int stepsPerRev, int microsteps, float maxRpm, bool reverse)
    : Valid {true}
    , StepsPerRev {stepsPerRev}
    , MicrostepsPerStep {microsteps}
    , MicrostepsPerCycle {MicrostepsPerStep * StepsPerCycle}
    , StepsPerDegree {(float)StepsPerRev / DegPerRev}
    , MicrostepsPerRev {StepsPerRev * MicrostepsPerStep}
    , MicrostepsPerDegree {(float)MicrostepsPerRev / DegPerRev}
    , MaxRpm {maxRpm}
    , MinimumUsPerStep { usPerStepFromRpm(MaxRpm) }
    , ReverseA {reverse}
  { }

  const bool Valid;
  const int StepsPerRev;
  const int MicrostepsPerStep;
  const int MicrostepsPerCycle;
  const float StepsPerDegree;
  const int MicrostepsPerRev;
  const float MicrostepsPerDegree;
  const float MaxRpm;
  const uint64_t MinimumUsPerStep;
  const bool ReverseA;

  int ustepsFromDegrees(float deg) const
  {
    return (int)(deg * MicrostepsPerDegree);
  }

  int stepsFromDegrees(float deg) const
  {
    return (int)(deg * StepsPerDegree);
  }

  double rpmFromUsPerStep(uint64_t usPerStep) const
  {
    return (double)MicrosecondsPerMinute / ((double)StepsPerRev * (double)usPerStep);
  }

  double rpmFromUsPerUstep(uint64_t usPerUstep) const
  {
    return (double)MicrosecondsPerMinute / ((double)MicrostepsPerRev * (double)usPerUstep);
  }

  uint64_t usPerStepFromRpm(double rpm) const
  {
    return (uint64_t)((double)MicrosecondsPerMinute / rpm / (double)StepsPerRev);
  }

  uint64_t usPerUstepFromRpm(double rpm) const
  {
    return (uint64_t)((double)MicrosecondsPerMinute / rpm / (double)MicrostepsPerRev);
  }

  int8_t normalizeStep(int8_t step) const
  {
    while (step < 0) step += MicrostepsPerCycle;
    return step % MicrostepsPerCycle;
  }
};

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
    Microstep = 2
  };

  virtual ~Stepper() = default;
  Stepper(const Stepper&) = delete;
  Stepper(Stepper&&) = delete;

  // Check if a move would be valid based on speed
  virtual int64_t getUsPerUstep(int delta, std::chrono::microseconds time) = 0;

  // Move the stepper by delta microsteps in the specified time
  // Returns false if this is not possible
  virtual bool move(int delta, std::chrono::microseconds time) = 0;

  // Move the stepper by delta microsteps at the specified RPM
  // Returns false if this is not possible
  bool move(int delta, float rpm = 50)
  {
    return move(delta, std::chrono::microseconds(motorInfo.usPerUstepFromRpm(rpm) * std::abs(delta)));
  }

  // Move the stepper by the specified number of degrees at the specified RPM
  // Returns false if this is not possible
  bool moveDegrees(float deg, float rpm = 50)
  {
    return move(motorInfo.ustepsFromDegrees(deg), rpm);
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

  // Power the coils for a given step position, [0, Microsteps per cycle)
  // Setting dir to something other than auto will allow the method to optimize
  // the update for movement in a direction.
  virtual void updateCoils(int8_t pos, bool forceUpdateAll = false) = 0;

  // Get the last position sent to updateCoils(...)
  virtual int8_t getLastPos() = 0;

  const StepperMotorInfo motorInfo;

protected:
  Stepper(StepperMotorInfo motorInfo) : motorInfo{motorInfo} {}
};