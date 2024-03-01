#pragma once

#include <pico/stdlib.h>
#include <hardware/gpio.h>

class Button
{
public:
  virtual ~Button() = default;

  bool pressed()
  {
    return state_;
  }

  uint32_t heldTimeMs()
  {
    if (!state_) return 0;
    return to_ms_since_boot(get_absolute_time()) - to_ms_since_boot(stateTime_);
  }

  uint32_t releasedTimeMs()
  {
    if (state_) return 0;
    return to_ms_since_boot(get_absolute_time()) - to_ms_since_boot(stateTime_);
  }

  bool heldActivate()
  {
    return holdActivate_;
  }

  bool buttonDown()
  {
    return state_ && !lastState_;
  }
  
  bool buttonUp()
  {
    return !state_ && lastState_;
  }

  void update()
  {
    lastState_ = state_;
    state_ = getButtonState();
    if (lastState_ != state_)
    {
      stateTime_ = get_absolute_time();
    }

    if (enableHoldAction_)
    {
      if (buttonDown())
      {
        holdActivationTime_ = make_timeout_time_ms(holdActivationMs_);
      }

      if (buttonUp() && holdSuppressButtonUp_)
      {
        lastState_ = state_;
        holdSuppressButtonUp_ = false;
        holdSuppressRepeat_ = false;
      }

      if (state_ && !holdSuppressRepeat_ && to_ms_since_boot(holdActivationTime_) <= to_ms_since_boot(get_absolute_time()))
      {
        holdActivate_ = true;
        holdActivationTime_ = make_timeout_time_ms(holdActivationRepeatMs_);
        holdSuppressButtonUp_ = true;
        holdSuppressRepeat_ = holdActivationRepeatMs_ < 0;
      }
      else
      {
        holdActivate_ = false;
      }
    }
  }

  void holdActivationRepeatMs(int val)
  {
    holdActivationRepeatMs_ = val;
  }

protected:
  Button(bool enableHoldAction = false) : enableHoldAction_(enableHoldAction) {}
  virtual bool getButtonState() = 0;
  bool state_;
  bool lastState_;
  absolute_time_t stateTime_;

  bool enableHoldAction_ = false;
  int holdActivationMs_ = 1000;
  int holdActivationRepeatMs_ = 0;
  absolute_time_t holdActivationTime_;
  bool holdActivate_ = false;
  bool holdSuppressButtonUp_ = false;
  bool holdSuppressRepeat_ = false;
};

class GPIOButton : public Button
{
public:
  GPIOButton(uint32_t pin, bool enableHoldAction = false, bool pullUp = true, bool pullDown = false, bool invert = true):
    Button(enableHoldAction),
    pin_(pin)
  {
    gpio_init(pin_);
    gpio_set_dir(pin_, GPIO_IN);
    if (pullUp)
    {
      gpio_pull_up(pin_);
    }
    if (pullDown)
    {
      gpio_pull_down(pin_);
    }
    if (invert)
    {
      gpio_set_inover(pin_, GPIO_OVERRIDE_INVERT);
    }
    else
    {
      gpio_set_inover(pin_, GPIO_OVERRIDE_NORMAL);
    }
    
    // Give the gpio set operations a chance to settle!
    sleep_until(make_timeout_time_ms(1));

    update();
    lastState_ = state_;
    stateTime_ = get_absolute_time();
  }

  virtual bool getButtonState() override
  {
    return gpio_get(pin_);
  }

  ~GPIOButton()
  {
    gpio_deinit(pin_);
  }

private:
  uint32_t pin_;
};