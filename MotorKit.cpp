#include "MotorKit.hpp"
#include "Logging.hpp"
#include <algorithm>
#include <cmath>
#include <memory>

# define M_PI           3.14159265358979323846

namespace {
// We are driving motors using a PCA9685 LED controller
// Below are some hardware mappings and info for this chip
constexpr int PCA9685ClockSpeed = 25000000; // 25 MHz
uint8_t getBaseAddrForChannel(uint8_t ch)
{
  // Per the PCA9685 datasheet
  return 0x06 + ch * 0x04;
}

template <typename... T>
void setDutyCycle(float dutyCycle, T&... reg)
{
  uint16_t onTime = 0;
  uint16_t offTime = dutyCycle * 4096.0;
  offTime = std::clamp(offTime, (uint16_t)0, (uint16_t)4096);

  // Special case for always on
  if (offTime == 4096) std::swap(onTime, offTime);

  (reg.setIfChanged(onTime, offTime),...);
}

template <typename... T>
void setDutyCycle(bool on, T&... reg)
{
  uint16_t onTime = on ? 4096 : 0;
  uint16_t offTime = on ? 0 : 4096;
  (reg.setIfChanged(onTime, offTime),...);
}

template <typename... T>
void setDutyCycle(uint16_t dutyCycle, T&... reg)
{
  uint16_t onTime = 0;
  uint16_t offTime = std::clamp(dutyCycle, (uint16_t)0, (uint16_t)4096);

  // Special case for always on
  if (offTime == 4096) std::swap(onTime, offTime);

  (reg.setIfChanged(onTime, offTime),...);
}
}

class MotorKitDC : public DCMotor
{
public:
  MotorKitDC(
    I2CRegister<uint16_t, uint16_t>& power, 
    I2CRegister<uint16_t, uint16_t>& forward, 
    I2CRegister<uint16_t, uint16_t>& backward) :
    power_{power},
    forward_{forward},
    backward_{backward},
    powered_{false}
  { }

  ~MotorKitDC()
  {
    powerOff();
  }

  // Set the motor power as a float -1 to 1
  void setSpeed(float power) override
  {
    if (!powered_)
    {
      powerOn();
    }
    if (power < 0.0f)
    {
      setDutyCycle(false, forward_);
      setDutyCycle(std::abs(power), backward_);
    }
    else if (power > 0.0f)
    {
      setDutyCycle(false, backward_);
      setDutyCycle(std::abs(power), forward_);
    }
    else
    {
      setDutyCycle(false, backward_, forward_);
    }
  }

  void powerOn()
  {
    powered_ = true;
    setDutyCycle(true, power_);
  }

  void powerOff()
  {
    powered_ = false;
    setDutyCycle(false, power_);
  }
private:
  bool powered_;
  I2CRegister<uint16_t, uint16_t>& power_;
  I2CRegister<uint16_t, uint16_t>& forward_;
  I2CRegister<uint16_t, uint16_t>& backward_;
};

class MotorKitStepper : public Stepper
{
public:
  MotorKitStepper(
    StepperMotorInfo motorInfo,
    I2CRegister<uint16_t, uint16_t>& coil1A, 
    I2CRegister<uint16_t, uint16_t>& coil2A, 
    I2CRegister<uint16_t, uint16_t>& coil1B, 
    I2CRegister<uint16_t, uint16_t>& coil2B, 
    I2CRegister<uint16_t, uint16_t>& dutyA, 
    I2CRegister<uint16_t, uint16_t>& dutyB )
    : Stepper(motorInfo)
    , grabbed_ {false}
    , stepMode_ {Mode::Single}
    , lastMicrostep_ {0}
    , motorPower_ {0.5f}
    , coil1A_{coil1A}
    , coil2A_{coil2A}
    , coil1B_{coil1B}
    , coil2B_{coil2B}
    , dutyCycleA_{dutyA}
    , dutyCycleB_{dutyB}
  {
    // Ensure motor is powered off on startup
    release();

    // Create the microstep curve
    microstepCurve_.resize(motorInfo.MicrostepsPerStep);
    for (int i=0; i < motorInfo.MicrostepsPerStep; ++i)
    {
      float t = (float)i / (float)motorInfo.MicrostepsPerStep;

      // Using sin(x) for microstep power is conventional
      microstepCurve_[i] = std::cos(t * M_PI / 2.0);
      
      // Some fun alternative functions that work worse
      // microstepCurve_[i] = std::sqrt(1.0f-t*t);  // Upper right quadrant of a circle
      // microstepCurve_[i] = t;  // Just plain linear interpolation
    }

    DEBUG_LOG("Connected stepper with max speed " << motorInfo.MinimumUsPerStep / motorInfo.MicrostepsPerStep << " per ustep (" << motorInfo.MaxRpm << " rpm)");
  }

  ~MotorKitStepper()
  {
    release();
  }

  int8_t snap(Direction dir) override
  {
    int8_t localMicrostep = lastMicrostep_ % motorInfo.MicrostepsPerStep;
    int8_t correction = 0;
    if (localMicrostep > 0)
    {
      if (dir == Direction::Forward || (dir == Direction::Auto && localMicrostep >= (motorInfo.MicrostepsPerStep / 2)))
      {
        correction = (int8_t)motorInfo.MicrostepsPerStep - (int8_t)localMicrostep;
      }
      else
      {
        correction =  -((int8_t)localMicrostep);
      }
    }
    updateCoils(motorInfo.normalizeStep(lastMicrostep_ + correction), true);
    return correction;
  }

  int64_t getUsPerUstep(int delta, std::chrono::microseconds time) override
  {
    if (delta == 0) return -1;
    uint64_t usPerMicrostepReq = time.count() / std::abs(delta);
    if (usPerMicrostepReq >= (motorInfo.MinimumUsPerStep / motorInfo.MicrostepsPerStep))
    {
      return usPerMicrostepReq;
    }
    return 0;
  }

  bool move(int delta, std::chrono::microseconds time) override
  {
    int64_t usPerMicrostepReq = getUsPerUstep(delta, time);

    if (usPerMicrostepReq == 0)
    {
      DEBUG_LOG("Too fast movement requested! Ignoring.");
      return false;
    }

    auto startTime = get_absolute_time();
    DEBUG_LOG("Move requested: " << delta << " usteps in " << std::chrono::duration_cast<std::chrono::milliseconds>(time).count() << " ms");
    DEBUG_LOG("Calculated " << usPerMicrostepReq << " us per ustep");

    int8_t moveStep = lastMicrostep_;
    absolute_time_t nextStepTime_ = make_timeout_time_us(usPerMicrostepReq);

    while (delta != 0)
    {
      if (delta > 0)
      {
        --delta;
        moveStep = motorInfo.normalizeStep(moveStep+1);
      }
      else // if (delta < 0)
      {
        ++delta;
        moveStep = motorInfo.normalizeStep(moveStep-1);
      }
      updateCoils(moveStep);
      busy_wait_until(nextStepTime_);
      nextStepTime_ = delayed_by_us(nextStepTime_, usPerMicrostepReq);
    }

    auto endTime = get_absolute_time();
    DEBUG_LOG("Move complete: " << absolute_time_diff_us(startTime, endTime) / 1000 << " ms");

    return true;
  }

  int8_t getLastPos() override
  {
    return lastMicrostep_;
  }

  void updateCoils(int8_t pos, bool forceUpdateAll = false) override
  {
    // Check to see that we're not trying to set an invalid coil state
    // if (pos >= motorInfo.MicrostepsPerCycle || pos < 0)
    // {
    //   DEBUG_LOG("Invalid step cycle given to update coils: " << (int)pos);
    //   return;
    // }

    int8_t microstep = (pos % motorInfo.MicrostepsPerStep);
    bool wholeStepTransition = (forceUpdateAll) ||
                               (pos == motorInfo.normalizeStep(lastMicrostep_+1) && microstep == 0) || 
                               (pos == motorInfo.normalizeStep(lastMicrostep_-1) && microstep == (motorInfo.MicrostepsPerStep-1));
    
    // Because we might end early, set the last microstep upfront, before any changes are made
    lastMicrostep_ = pos;

    // If we aren't microstepping, we don't have to do anything at all
    // unless we're changing whole steps.
    if (stepMode_ != Mode::Microstep && !wholeStepTransition)
    {
      return;
    }

    // Choose the role of each coil based on what part of the step cycle we are in
    I2CRegister<uint16_t, uint16_t>* begin, * end, * offA, * offB;
    if (pos < (motorInfo.MicrostepsPerStep * 1))
    {
      begin = &coil2B_;
      end = &coil1A_;
      offA = &coil2A_;
      offB = &coil1B_;
    }
    else if (pos < (motorInfo.MicrostepsPerStep * 2))
    {
      begin = &coil1A_;
      end = &coil1B_;
      offA = &coil2A_;
      offB = &coil2B_;
    }
    else if (pos < (motorInfo.MicrostepsPerStep * 3))
    {
      begin = &coil1B_;
      end = &coil2A_;
      offA = &coil1A_;
      offB = &coil2B_;
    }
    else
    {
      begin = &coil2A_;
      end = &coil2B_;
      offA = &coil1A_;
      offB = &coil1B_;
    }
    
    // Calculate how much power should go to begin and end
    float powerBegin, powerEnd;
    if (stepMode_ == Mode::Single)
    {
      powerBegin = 0.0f * motorPower_;
      powerEnd = 1.0f * motorPower_;
    }
    else if (stepMode_ == Mode::Double)
    {
      powerBegin = 1.0f * motorPower_;
      powerEnd = 1.0f * motorPower_;
    }
    else if (stepMode_ == Mode::Microstep)
    {
      powerBegin = microstepCurve_[microstep] * motorPower_;
      powerEnd = microstepCurve_[(motorInfo.MicrostepsPerStep - 1) - microstep] * motorPower_;
    }

    // Send the power to the motors if unpowered
    if (!grabbed_) grab();

    // Which coils are wholly off only changes at the whole step transitions
    if (wholeStepTransition) setDutyCycle(false, *offA, *offB);
    setDutyCycle(powerBegin, *begin);
    setDutyCycle(powerEnd, *end);
  }

  void grab() override
  {
    setDutyCycle(true, dutyCycleA_, dutyCycleB_);
    grabbed_ = true;
  }

  void release() override
  {
    grabbed_ = false;
    setDutyCycle(false, coil1A_, coil2A_, coil1B_, coil2B_, dutyCycleA_, dutyCycleB_);
  }

  void setPower(float power) override
  {
    motorPower_ = power;
    if (grabbed_) updateCoils(lastMicrostep_, true);
  }

  void setMode(Mode stepMode) override
  {
    stepMode_ = stepMode;
    if (grabbed_) updateCoils(lastMicrostep_, true);
  }
  
private:
  bool grabbed_;
  Mode stepMode_;
  int8_t lastMicrostep_;
  float motorPower_;
  std::vector<float> microstepCurve_;

  // PWM registers
  I2CRegister<uint16_t, uint16_t>& coil1A_;
  I2CRegister<uint16_t, uint16_t>& coil2A_;
  I2CRegister<uint16_t, uint16_t>& coil1B_;
  I2CRegister<uint16_t, uint16_t>& coil2B_;
  I2CRegister<uint16_t, uint16_t>& dutyCycleA_;
  I2CRegister<uint16_t, uint16_t>& dutyCycleB_;
};

MotorKit::MotorKit(I2CInterface& i2c, uint8_t devAddr, double pwmFrequency)
  : mode1_(i2c, devAddr, 0x00)
  , mode2_(i2c, devAddr, 0x01)
  , prescale_(i2c, devAddr, 0xFE)
{
  DEBUG_LOG("Setting up MotorKit board...");

  // Init the PCA9685
  uint8_t origMode1;
  DEBUG_LOG("Resetting PCA9685 chip...");
  mode1_.set(0x00);
  mode1_.get(origMode1);
  DEBUG_LOG("Got mode1 as: 0x" << std::hex << (int)origMode1 << std::dec);

  // Put chip to sleep
  uint8_t sleepMode1 = (origMode1 & 0x7F) | 0x10;
  DEBUG_LOG("Sleeping chip to update prescale: 0x" << std::hex << (int)sleepMode1 << std::dec);
  mode1_.set(sleepMode1);

  // Calculate frequency prescale
  uint8_t prescale = (uint8_t)((double)PCA9685ClockSpeed / 4096.0 / pwmFrequency + 0.5) - 1;
  DEBUG_LOG("Setting prescale to: " << (int)prescale);
  prescale_.set(prescale);

  // Restore original mode
  DEBUG_LOG("Restoring mode1 to 0x" << std::hex << (int)origMode1 << std::dec);
  mode1_.set(origMode1);
  sleep_ms(5);

  // Enable auto-increment
  uint8_t autoIncMode1 = origMode1 | 0xA0;
  DEBUG_LOG("Set mode1 to autoincrement: 0x" << std::hex << (int)autoIncMode1 << std::dec);
  mode1_.set(autoIncMode1);

  // Setup all the PWM registers
  for (int i=0; i < 16; ++i)
  {
    pwmChannels_.emplace_back(i2c, devAddr, getBaseAddrForChannel(i));
  }
}

Stepper* MotorKit::connectStepper(int id, StepperMotorInfo info)
{
  constexpr int pwm[][6] 
  {
    {10,9,11,12,8,13},  // Stepper 0 channels
    {4,3,5,6,7,2}       // Stepper 1 channels
  };

  // Check valid id
  if (id < 0 || id > 2)
  {
    DEBUG_LOG("Connect failure: stepper id " << id << " out of range");
    return nullptr;
  }

  // Check stepper conflicts
  if (steppers_[id])
  {
    DEBUG_LOG("Connect failure: stepper id " << id << " already connected");
    return nullptr;
  }

  // Check DC motor conflicts
  if (dcs_[id * 2] || dcs_[id * 2 + 1])
  {
    DEBUG_LOG("Connect failure: stepper id " << id << " conflicts with connected DC motor");
    return nullptr;
  }

  // Make the stepper
  steppers_[id] = std::make_unique<MotorKitStepper>(
    info,
    pwmChannels_[pwm[id][info.ReverseA ? 1 : 0]], 
    pwmChannels_[pwm[id][info.ReverseA ? 0 : 1]], 
    pwmChannels_[pwm[id][2]], 
    pwmChannels_[pwm[id][3]], 
    pwmChannels_[pwm[id][4]], 
    pwmChannels_[pwm[id][5]]
  );
  return getStepper(id);
}

void MotorKit::disconnectStepper(int id)
{
  if (id >= 0 && id < 2 && !steppers_[id])
  {
    steppers_[id].reset();
  }
  else
  {
    DEBUG_LOG("Discconet failure: stepper " << id << " does not exist!");
  }
}

Stepper* MotorKit::getStepper(int id)
{
  if (id >= 0 && id < 2 && steppers_[id])
  {
    return steppers_[id].get();
  }
  DEBUG_LOG("Get failure: stepper " << id << " does not exist!");
  return nullptr;
}

DCMotor* MotorKit::connectDc(int id, StepperMotorInfo info)
{
  constexpr int pwm[][3] 
  {
    {8, 9, 10},
    {13, 11, 12},
    {2, 3, 4},
    {7, 5, 6}
  };

  // Check valid id
  if (id < 0 || id > 4)
  {
    DEBUG_LOG("Connect failure: DC motor id " << id << " out of range!");
    return nullptr;
  }

  // Check motor conflicts
  if (dcs_[id])
  {
    DEBUG_LOG("Connect failure: dc motor id " << id << " already connected!");
    return nullptr;
  }

  // Look for stepper conflicts
  int conflictingStepperId = id / 2;
  if (steppers_[conflictingStepperId])
  {
    DEBUG_LOG("Connect failure: dc motor id " << id << " connot be constructed because a stepper is using its resources.");
    return nullptr;
  }

  // Make the DC motor
  dcs_[id] = std::make_unique<MotorKitDC>(
    pwmChannels_[pwm[id][0]], 
    pwmChannels_[pwm[id][1]], 
    pwmChannels_[pwm[id][2]]);
  return dcs_[id].get();
}

void MotorKit::disconnectDc(int id)
{
  if (id >= 0 && id < 4 && !dcs_[id])
  {
    dcs_[id].reset();
  }
  else
  {
    DEBUG_LOG("Disconnect failure: dc motor id " << id << " does not exist!");
  }
}

DCMotor* MotorKit::getDc(int id)
{
  if (id >= 0 && id < 4 && !dcs_[id])
  {
    return dcs_[id].get();
  }
  DEBUG_LOG("Get failure: dc motor id " << id << " does not exist!");
  return nullptr;
}