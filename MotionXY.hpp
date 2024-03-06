#pragma once

#include "Stepper.hpp"
#include "Button.hpp"
#include <cmath>
#include <deque>

class MotionXY
{
  struct MoveXY
  {
    Stepper& stepperU;
    Stepper& stepperV;
    int64_t& u;
    int64_t& v;
    int64_t endU;
    int64_t endV;
    uint64_t usPerUstepU;
    uint64_t usPerUstepV;
    absolute_time_t nextStepTime;
    uint64_t accumV{0};

    bool stepMove()
    {
      // Figure out which stepper to step
      busy_wait_until(nextStepTime);
      accumV += usPerUstepU;
      if (endU != u)
      {
        int8_t dir = endU > u ? 1 : -1;
        int8_t pos = stepperU.motorInfo.normalizeStep(stepperU.getLastPos() + dir);
        stepperU.updateCoils(pos);
        u += dir;
      }
      if (endV != v && accumV >= usPerUstepV)
      {
        int8_t dir = endV > v ? 1 : -1;
        int8_t pos = stepperV.motorInfo.normalizeStep(stepperV.getLastPos() + dir);
        stepperV.updateCoils(pos);
        v += dir;
        accumV -= usPerUstepV;
      }
      nextStepTime = delayed_by_us(nextStepTime, usPerUstepU);

      return (endU == u && endV == v);
    }
  };
public:
  MotionXY( Stepper& stepperX, Stepper& stepperY, 
            Button& limX, Button& limY, 
            double stepsPerMmX, double stepsPerMmY,
            double homeOffsetX, double homeOffsetY, double homeSpeedMmPerSec,
            double sizeMmX, double sizeMmY)
    : stepperX_{stepperX}
    , stepperY_{stepperY}
    , limX_{limX}
    , limY_{limY}
    , homeOffsetX_{homeOffsetX}
    , homeOffsetY_{homeOffsetY}
    , homeSpeedMmPerSec_ {homeSpeedMmPerSec}
    , stepsPerMmX_{stepsPerMmX}
    , stepsPerMmY_{stepsPerMmY}
    , x_{0}
    , y_{0}
    , sizeMmX_{sizeMmX}
    , sizeMmY_{sizeMmY}
    , sizeUstepX_{(uint64_t)(sizeMmX_ * stepsPerMmX_ * stepperX_.motorInfo.MicrostepsPerStep)}
    , sizeUstepY_{(uint64_t)(sizeMmY_ * stepsPerMmY_ * stepperY_.motorInfo.MicrostepsPerStep)}
    , homed_{false}
  {}

  double getMmFromUstepsX(int64_t x)
  {
    return (double)x / stepsPerMmX_ / (double)stepperX_.motorInfo.MicrostepsPerStep;
  }

  double getMmFromUstepsY(int64_t y)
  {
    return (double)y / stepsPerMmY_ / (double)stepperY_.motorInfo.MicrostepsPerStep;
  }

  double getLastQueuedPosX()
  {
    if (queuedMoves_.empty())
    {
      return x_;
    }
    auto& lastMove = queuedMoves_.at(queuedMoves_.size()-1);
    if (&lastMove.stepperU == &stepperX_)
    {
      return lastMove.endU;
    }
    else
    {
      return lastMove.endV;
    }
  }

  double getLastQueuedPosY()
  {
    if (queuedMoves_.empty())
    {
      return y_;
    }
    auto& lastMove = queuedMoves_.at(queuedMoves_.size()-1);
    if (&lastMove.stepperU == &stepperX_)
    {
      return lastMove.endV;
    }
    else
    {
      return lastMove.endU;
    }
  }
  
  // Set the move destination to the specified coordinates
  // Just schedules a move. Call stepMove or completeMove 
  // to actually perform it.
  bool moveTo(double xMm, double yMm, double mmPerSec)
  {
    if (!homed_)
    {
      DEBUG_LOG("Got move cmd while not homed, ignoring.");
      return false;
    }

    // Get the final position in steps and bounds check it
    int64_t endX = (int64_t)(stepsPerMmX_ * xMm * stepperX_.motorInfo.MicrostepsPerStep);
    int64_t endY = (int64_t)(stepsPerMmY_ * yMm * stepperY_.motorInfo.MicrostepsPerStep);
    if (endX < 0 || endX >= sizeUstepX_ || endY < 0 || endY >= sizeUstepY_)
    {
      DEBUG_LOG("Move out of bounds requested, ignoring.");
      return false;
    }

    // Calculate the desired move time
    int64_t dX = endX - getLastQueuedPosX();
    int64_t dY = endY - getLastQueuedPosY();
    double dXMm = getMmFromUstepsX(dX);
    double dYMm = getMmFromUstepsY(dY);
    double distMm = std::sqrt(std::pow(dXMm, 2.0) + std::pow(dYMm, 2.0));
    uint64_t moveTimeUs = (uint64_t)(distMm / mmPerSec / 1000000.0);

    // Get the requested speed of each axis and make sure it's not too fast
    uint64_t usPerUstepX = stepperX_.getUsPerUstep(dX, std::chrono::microseconds(moveTimeUs));
    uint64_t usPerUstepY = stepperY_.getUsPerUstep(dY, std::chrono::microseconds(moveTimeUs));
    if (usPerUstepX == 0 || usPerUstepY == 0)
    {
      DEBUG_LOG("Too fast movement requested! Ignoring.");
      return false;
    }

    // Push this move on to the stack and return true
    // to indicate it has been sucessfully enqueued
    if (usPerUstepX < usPerUstepY)
    {
      queuedMoves_.push_back(
      {
        .stepperU = stepperX_,
        .stepperV = stepperY_,
        .u = x_,
        .v = y_,
        .endU = endX,
        .endV = endY,
        .usPerUstepU = usPerUstepX,
        .usPerUstepV = usPerUstepY,
        .nextStepTime = get_absolute_time()
      });
    }
    else
    {
      queuedMoves_.push_back(
      {
        .stepperU = stepperY_,
        .stepperV = stepperX_,
        .u = y_,
        .v = x_,
        .endU = endY,
        .endV = endX,
        .usPerUstepU = usPerUstepY,
        .usPerUstepV = usPerUstepX,
        .nextStepTime = get_absolute_time()
      });
    }
    return true;
  }

  // Run this method in a loop to complete all enqueued moves
  // It will block to throttle steps to the correct rate
  // Returns true when all scheduled moves are complete
  bool stepMove()
  {
    if (queuedMoves_.empty())
    {
      return true;
    }
    if (queuedMoves_[0].stepMove())
    {
      queuedMoves_.pop_front();
    }
    return queuedMoves_.empty();
  }

  // Calls stepMove repeatedly until the current move is complete.
  void completeAllMoves()
  {
    while (!stepMove())
    {
      tight_loop_contents();
    }
  }

  void stop()
  {
    queuedMoves_.clear();
  }

  // Performs a homing process
  bool home()
  {
    stop();
    homed_ =  homeAxis(stepperX_, limX_,  stepsPerMmX_, homeOffsetX_, sizeMmX_) &&
              homeAxis(stepperY_, limY_,  stepsPerMmY_, homeOffsetY_, sizeMmY_);
    return homed_;
  }

private:
  static constexpr double MaxBackoffSteps = 20.0;
  bool homeAxis(Stepper& stepper, Button& lim, double stepsPerMm, double homeOffset, double sizeMm)
  {
    // Figure out how many us between microsteps for homing
    uint64_t usPerMicrostep =  (uint64_t)(1000000.0 / (homeSpeedMmPerSec_ * stepsPerMm * (double)stepper.motorInfo.MicrostepsPerStep));
    DEBUG_LOG("Home operation started: speed is " << usPerMicrostep << " us per ustep");

    // Setup some movement vars
    int8_t ustep = stepper.getLastPos();
    absolute_time_t nextStepTime = make_timeout_time_us(usPerMicrostep);
    uint64_t stepsRemaining;

    // Move the stepper backwards until the limit switch is pressed
    stepsRemaining = (uint64_t) (sizeMm * 1.2 * stepsPerMm * stepper.motorInfo.MicrostepsPerStep);
    while (!lim.pressed() && stepsRemaining > 0)
    {
      --stepsRemaining;
      ustep = stepper.motorInfo.normalizeStep(ustep - 1);
      stepper.updateCoils(ustep);
      busy_wait_until(nextStepTime);
      nextStepTime = delayed_by_us(nextStepTime, usPerMicrostep);
    }
    if (stepsRemaining == 0)
    {
      // Power down the malfunctioning stepper and return false for failure
      DEBUG_LOG("Home operation failed: home seek limit exceeded without triggering limit switch");
      stepper.release();
      return false;
    }

    // Back off the limit switch until it's not pressed
    stepsRemaining = (uint64_t)(MaxBackoffSteps * stepper.motorInfo.MicrostepsPerStep);
    while (lim.pressed() && stepsRemaining > 0)
    {
      --stepsRemaining;
      ustep = stepper.motorInfo.normalizeStep(ustep + 1);
      stepper.updateCoils(ustep);
      busy_wait_until(nextStepTime);
      nextStepTime = delayed_by_us(nextStepTime, usPerMicrostep);
    }
    if (stepsRemaining == 0)
    {
      // Power down the malfunctioning stepper and return false for failure
      DEBUG_LOG("Home operation failed: backoff limit exceeded without releasing limit switch");
      stepper.release();
      return false;
    }

    // Execute a move to origin
    stepsRemaining = homeOffset * stepsPerMm * stepper.motorInfo.MicrostepsPerStep;
    while (stepsRemaining > 0)
    {
      --stepsRemaining;
      ustep = stepper.motorInfo.normalizeStep(ustep + 1);
      stepper.updateCoils(ustep);
      busy_wait_until(nextStepTime);
      nextStepTime = delayed_by_us(nextStepTime, usPerMicrostep);
    }

    // Set the internally held position to 0,0
    x_ = 0;
    y_ = 0;

    // Homed successfully!
    return true;
  }

  Stepper& stepperX_;
  Stepper& stepperY_;
  Button& limX_;
  Button& limY_;
  double homeOffsetX_;
  double homeOffsetY_;
  double homeSpeedMmPerSec_;
  double stepsPerMmX_;
  double stepsPerMmY_;
  int64_t x_;
  int64_t y_;
  double sizeMmX_;
  double sizeMmY_;
  uint64_t sizeUstepX_;
  uint64_t sizeUstepY_;
  bool homed_;
  std::deque<MoveXY> queuedMoves_;
};