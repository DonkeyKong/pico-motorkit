#pragma once

#include <pico/stdlib.h>
#include <hardware/gpio.h>

class DiscreteOut
{
public:
  DiscreteOut(uint32_t pin, bool pullUp = false, bool pullDown = true, bool invert = false) :
    pin_(pin)
  {
    gpio_init(pin_);
    gpio_set_dir(pin_, GPIO_OUT);
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
      gpio_set_outover(pin_, GPIO_OVERRIDE_INVERT);
    }
    else
    {
      gpio_set_outover(pin_, GPIO_OVERRIDE_NORMAL);
    }

    set(state_);
    
    // Give the gpio set operations a chance to settle!
    sleep_until(make_timeout_time_ms(1));
  }
  void set(bool state)
  {
    gpio_put(pin_, state);
    state_ = state;
  }
  bool get()
  {
    return state_;
  }
  ~DiscreteOut()
  {
    gpio_deinit(pin_);
  }
private:
  uint32_t pin_;
  bool state_ = false;
};