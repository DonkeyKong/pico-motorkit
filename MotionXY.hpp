#pragma once

#include "Stepper.hpp"
#include "Button.hpp"
#include <cmath>
#include <deque>



class Stage2D
{
  template <typename T>
  inline T dist(T dX, T dY)
  {
    return std::sqrt(std::pow(dX, (T)2) + std::pow(dY, (T)2));
  }

  template <typename T>
  inline bool between(T val, T bound1, T bound2)
  {
    if (bound1 > bound2) return val >= bound2 && val <= bound1;
    return val >= bound1 && val <= bound2;
  }

  struct Move2Axis
  {
    Stepper& stepperS;
    Stepper& stepperT;
    int64_t& s;
    int64_t& t;
    int64_t endS;
    int64_t endT;
    int64_t usPerUstepS;
    int64_t usPerUstepT;
    absolute_time_t nextStepTime;
    double moveTimeSeconds{0}; // informational only
    uint64_t accumT{0};
    bool firstStep{true};

    bool stepMove()
    {
      // Figure out which stepper to step
      busy_wait_until(nextStepTime);
      accumT += usPerUstepS;
      if (endS != s)
      {
        int8_t dir = endS > s ? 1 : -1;
        int8_t pos = stepperS.motorInfo.normalizeStep(stepperS.getLastPos() + dir);
        stepperS.updateCoils(pos, firstStep);
        s += dir;
      }
      if (endT != t && accumT >= usPerUstepT)
      {
        int8_t dir = endT > t ? 1 : -1;
        int8_t pos = stepperT.motorInfo.normalizeStep(stepperT.getLastPos() + dir);
        stepperT.updateCoils(pos, firstStep);
        t += dir;
        accumT -= usPerUstepT;
      }
      else if (firstStep)
      {
        stepperT.updateCoils(stepperT.getLastPos(), true);
      }
      firstStep = false;
      nextStepTime = delayed_by_us(nextStepTime, usPerUstepS);

      return (endS == s && endT == t);
    }
  };
public:

  enum class StageMoveMode
  {
    Hold,
    Release
  };

  Stage2D(Stepper& stepperU, Stepper& stepperV, 
          Button& limX, Button& limY,
          double sizeX, double sizeY,
          double homeOffsetX, double homeOffsetY, double homeSpeedMmPerSec)
    : stepperU_{stepperU}, stepperV_{stepperV}
    , limX_{limX}, limY_{limY}
    , sizeX{sizeX}, sizeY{sizeY}
    , homeOffsetX_{homeOffsetX}, homeOffsetY_{homeOffsetY}, homeSpeedMmPerSec_ {homeSpeedMmPerSec}
    , u_{0}, v_{0}, homed_{false}
  {
  }

  void setMode(StageMoveMode mode)
  {
    mode_ = mode;
  }

  void forceHomed(int64_t u = 0, int64_t v = 0)
  {
    u_ = u;
    v_ = v;
    homed_ = true;
  }

  int64_t getLastQueuedPosU()
  {
    if (queuedMoves_.empty())
    {
      return u_;
    }
    auto& lastMove = queuedMoves_.at(queuedMoves_.size()-1);
    if (&lastMove.stepperS == &stepperU_)
    {
      return lastMove.endS;
    }
    else
    {
      return lastMove.endT;
    }
  }

  int64_t getLastQueuedPosV()
  {
    if (queuedMoves_.empty())
    {
      return v_;
    }
    auto& lastMove = queuedMoves_.at(queuedMoves_.size()-1);
    if (&lastMove.stepperS == &stepperU_)
    {
      return lastMove.endT;
    }
    else
    {
      return lastMove.endS;
    }
  }

  // Set the move destination to the specified relative coordinates
  // The coordinates are relative to the stage position or last 
  // scheduled move end position if any exist.
  // Just schedules a move. Call stepMove or completeAllMoves
  // to actually perform it.
  bool moveRel(double x, double y, double mmPerSec)
  {
    double startX, startY;
    xyFromUv(startX, startY, getLastQueuedPosU(), getLastQueuedPosV());
    return moveTo(startX + x, startY + y, mmPerSec);
  }
  
  // Set the move destination to the specified coordinates
  // Just schedules a move. Call stepMove or completeAllMoves
  // to actually perform it.
  bool moveTo(double x, double y, double mmPerSec)
  {
    if (!homed_)
    {
      DEBUG_LOG("Got move cmd while not homed, ignoring.");
      return false;
    }

    // Get the final position in steps and bounds check it
    int64_t endU, endV;
    uvFromXy(endU, endV, x, y);
    
    if (!between(x, 0.0, sizeX) || !between(y, 0.0, sizeY))
    {
      DEBUG_LOG("Move out of bounds requested, ignoring.");
      DEBUG_LOG("    End Pos: ( " << x << " , " << y << " ) mm");
      DEBUG_LOG("    Size : ( " << sizeX << " , " << sizeY << " ) mm");
      DEBUG_LOG("Move out of bounds requested, ignoring.");
      return false;
    }

    // Calculate the desired move time
    int64_t dU = endU - getLastQueuedPosU();
    int64_t dV = endV - getLastQueuedPosV();
    double dX, dY;
    xyFromUv(dX, dY, dU, dV);
    double distMm = dist(dX, dY);
    uint64_t moveTimeUs = (uint64_t)(distMm / mmPerSec * 1000000.0);
    DEBUG_LOG("Requested move to UV pos ( " << endU << " , " << endV << " ) in " << (distMm / mmPerSec) << " sec");

    // Get the requested speed of each axis and make sure it's not too fast
    int64_t usPerUstepU = stepperU_.getUsPerUstep(dU, std::chrono::microseconds(moveTimeUs));
    int64_t usPerUstepV = stepperV_.getUsPerUstep(dV, std::chrono::microseconds(moveTimeUs));
    if (usPerUstepU == 0 || usPerUstepV == 0)
    {
      DEBUG_LOG("Too fast movement requested! Ignoring.");
      return false;
    }

    if (usPerUstepU == -1 && usPerUstepV == -1)
    {
      DEBUG_LOG("Movement does not go anywhere! Ignoring.");
      return false;
    }

    bool primaryAxisU = (usPerUstepV == -1 || usPerUstepU < usPerUstepV);

    // Push this move on to the stack and return true
    // to indicate it has been sucessfully enqueued
    queuedMoves_.push_back(
    {
      .stepperS = primaryAxisU ? stepperU_ : stepperV_,
      .stepperT = primaryAxisU ? stepperV_ : stepperU_,
      .s = primaryAxisU ? u_ : v_,
      .t = primaryAxisU ? v_ : u_,
      .endS = primaryAxisU ? endU : endV,
      .endT = primaryAxisU ? endV : endU,
      .usPerUstepS = primaryAxisU ? usPerUstepU : usPerUstepV,
      .usPerUstepT = primaryAxisU ? usPerUstepV : usPerUstepU,
      .nextStepTime = get_absolute_time(),
      .moveTimeSeconds = (distMm / mmPerSec)
    });
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
      // Transfer the next step time to the first step of the next move
      if (queuedMoves_.size() > 1)
      {
        queuedMoves_[1].nextStepTime = queuedMoves_[0].nextStepTime;
      }
      queuedMoves_.pop_front();
    }
    if (queuedMoves_.empty() && mode_ == StageMoveMode::Release)
    {
      releaseSteppers();
    }
    return queuedMoves_.empty();
  }

  // Calls stepMove repeatedly until the current move is complete.
  void completeAllMoves()
  {
    uint64_t usteps = 0;
    double totalEstTime = 0;
    for (auto& move : queuedMoves_)
    {
      totalEstTime += move.moveTimeSeconds;
    }
    DEBUG_LOG("Queued moves should take " <<  totalEstTime << " sec");
    auto startTime = get_absolute_time();

    while (!stepMove())
    {
      ++usteps;
    }

    int64_t elapsedTimeUs = absolute_time_diff_us(startTime, get_absolute_time());
    DEBUG_LOG("Completed " << usteps << " usteps in " <<  (double)elapsedTimeUs / 1000000.0 << " sec");
  }

  void stop()
  {
    queuedMoves_.clear();
    if (mode_ == StageMoveMode::Release) releaseSteppers();
  }

  //Performs a homing process
  bool home()
  {
    stop();
    homed_ =  homeAxis(1, 0, limX_) &&
              homeAxis(0, 1, limY_);
    moveTo(homeOffsetX_, homeOffsetY_, homeSpeedMmPerSec_);
    completeAllMoves();
    u_ = 0;
    v_ = 0;
    return homed_;
  }

  void releaseSteppers()
  {
    stepperU_.release();
    stepperV_.release();
  }

protected:

  virtual void xyFromUv(double& x, double& y, int64_t u, int64_t v) = 0;
  virtual void uvFromXy(int64_t& u, int64_t& v, double x, double y) = 0;

  static constexpr int BackoffSteps = 128;
  bool homeAxis(double xCoeff, double yCoeff, Button& lim)
  {
    // The homing movement is the [size of the stage] * -1.2
    double endX = xCoeff * sizeX * -1.2;
    double endY = yCoeff * sizeY * -1.2;
    int64_t endU, endV;
    uvFromXy(endU, endV, endX, endY);
    double endTimeUs = dist(endX, endY) / homeSpeedMmPerSec_ * 1000000.0;
    
    int64_t usPerUstepU = stepperU_.getUsPerUstep(endU, std::chrono::microseconds((int64_t)endTimeUs));
    int64_t usPerUstepV = stepperV_.getUsPerUstep(endV, std::chrono::microseconds((int64_t)endTimeUs));
    if (usPerUstepU == 0 || usPerUstepV == 0)
    {
      DEBUG_LOG("Home operation failed: home seek speed too fast!");
      return false;
    }
    if (usPerUstepU == -1 && usPerUstepV == -1)
    {
      DEBUG_LOG("Home operation failed: no axis will move");
      return false;
    }
    bool primaryAxisU = (usPerUstepV == -1 || usPerUstepU < usPerUstepV);

    // Create a move for the home action
    u_ = 0;
    v_ = 0;
    Move2Axis homeMove 
    {
      .stepperS = primaryAxisU ? stepperU_ : stepperV_,
      .stepperT = primaryAxisU ? stepperV_ : stepperU_,
      .s = primaryAxisU ? u_ : v_,
      .t = primaryAxisU ? v_ : u_,
      .endS = primaryAxisU ? endU : endV,
      .endT = primaryAxisU ? endV : endU,
      .usPerUstepS = primaryAxisU ? usPerUstepU : usPerUstepV,
      .usPerUstepT = primaryAxisU ? usPerUstepV : usPerUstepU,
      .nextStepTime = get_absolute_time(),
      .moveTimeSeconds = dist(endX, endY) / homeSpeedMmPerSec_
    };

    DEBUG_LOG("Home operation started: speed is " << homeMove.usPerUstepS << " us per ustep (primary)");
    lim.update();
    while (!lim.pressed() && !homeMove.stepMove())
    {
      lim.update();
    }
    if (!lim.pressed())
    {
      DEBUG_LOG("Home operation failed: home seek limit exceeded without triggering limit switch");
      releaseSteppers();
      return false;
    }

    // Back off from the limit switch
    u_ = 0;
    v_ = 0;
    homeMove.endS = -homeMove.endS;
    homeMove.endT = -homeMove.endT;
    homeMove.accumT = 0; // This barely matters but do it anyway
    int stepsRemaining = BackoffSteps;
    lim.update();
    while (stepsRemaining > 0 && !homeMove.stepMove() && lim.pressed())
    {
      --stepsRemaining;
      lim.update();
    }
    if (lim.pressed())
    {
      DEBUG_LOG("Home operation failed: backoff limit exceeded without releasing limit switch");
      releaseSteppers();
      return false;
    }

    // Set the internally held position to 0,0
    u_ = 0;
    v_ = 0;

    // Homed successfully!
    return true;
  }

  Stepper& stepperU_;
  Stepper& stepperV_;
  Button& limX_;
  Button& limY_;
public:
  const double sizeX;
  const double sizeY;
private:
  double homeOffsetX_;
  double homeOffsetY_;
  double homeSpeedMmPerSec_;
  int64_t u_;
  int64_t v_;
  bool homed_;
  StageMoveMode mode_ {StageMoveMode::Hold};
  std::deque<Move2Axis> queuedMoves_;
};

class StageXY final : public Stage2D
{
public:
  StageXY(Stepper& stepperU, Stepper& stepperV, 
          Button& limX, Button& limY,
          double sizeX, double sizeY,
          double homeOffsetX, double homeOffsetY, double homeSpeedMmPerSec,
          double stepsUPerMmX, double stepsVPerMmY)
           : Stage2D(stepperU, stepperV, 
           limX,  limY,
           sizeX, sizeY,
           homeOffsetX,  homeOffsetY,  homeSpeedMmPerSec),
           stepsPerMmX_{stepsUPerMmX},
           stepsPerMmY_{stepsVPerMmY}
  {
  }

protected:

  void xyFromUv(double& x, double& y, int64_t u, int64_t v) override
  {
    x = (double)u / stepsPerMmX_ / (double)stepperU_.motorInfo.MicrostepsPerStep;
    y = (double)v / stepsPerMmY_ / (double)stepperV_.motorInfo.MicrostepsPerStep;
  }

  void uvFromXy(int64_t& u, int64_t& v, double x, double y) override
  {
    u = x * stepsPerMmX_ * stepperU_.motorInfo.MicrostepsPerStep;
    v = y * stepsPerMmY_ * stepperV_.motorInfo.MicrostepsPerStep;
  }

private:
  double stepsPerMmX_;
  double stepsPerMmY_;
};

class StageCoreXY final : public Stage2D
{
public:
  StageCoreXY(Stepper& stepperU, Stepper& stepperV, 
          Button& limX, Button& limY,
          double sizeX, double sizeY,
          double homeOffsetX, double homeOffsetY, double homeSpeedMmPerSec,
          double stepsUPerMmA = 0, double stepsVPerMmB = 0) // reflects the size of generic GT2 pulleys
           : Stage2D(stepperU, stepperV, 
           limX,  limY,
           sizeX, sizeY,
           homeOffsetX,  homeOffsetY,  homeSpeedMmPerSec),
           stepsPerMmA_{stepsUPerMmA},
           stepsPerMmB_{stepsVPerMmB}
  {
  }

protected:

  void xyFromUv(double& x, double& y, int64_t u, int64_t v) override
  {
    double a = (double)u / stepsPerMmA_ / (double)stepperU_.motorInfo.MicrostepsPerStep;
    double b = (double)v / stepsPerMmB_ / (double)stepperV_.motorInfo.MicrostepsPerStep;
    x = 0.5 * (a + b);
    y = 0.5 * (a - b);
  }

  void uvFromXy(int64_t& u, int64_t& v, double x, double y) override
  {
    double a = x + y;
    double b = x - y;
    u = a * stepsPerMmA_ * (double)stepperU_.motorInfo.MicrostepsPerStep;
    v = b * stepsPerMmB_ * (double)stepperV_.motorInfo.MicrostepsPerStep;
  }

private:
  double stepsPerMmA_;
  double stepsPerMmB_;
};