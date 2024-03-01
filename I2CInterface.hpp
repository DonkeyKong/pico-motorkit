#pragma once

#include "Logging.hpp"

#include <hardware/gpio.h>
#include <hardware/i2c.h>
//#include <pico/binary_info.h>

#include <vector>
#include <cstring>


class I2CInterface
{
public:
  I2CInterface(i2c_inst_t* i2c, int sdaPin, int sclPin, int baud = 100 * 1000) :
  i2c_ {i2c}
  {
    auto actualBaudRate = i2c_init(i2c, baud);
    DEBUG_LOG("Requested i2c with baud rate " << baud << ", initialized with rate " << actualBaudRate);
    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sclPin, GPIO_FUNC_I2C);
    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);
    DEBUG_LOG("i2c init complete!");
  }

  ~I2CInterface()
  {
    i2c_deinit(i2c_);
    DEBUG_LOG("i2c closed");
  }

  // Attempt to write specified number of bytes to address, blocking until the specified absolute time is reached.
  int write_blocking_until(uint8_t addr, const uint8_t* src, size_t len, bool nostop, absolute_time_t until)
  {
    return i2c_write_blocking_until(i2c_, addr, src, len, nostop, until);
  }

  //  Attempt to read specified number of bytes from address, blocking until the specified absolute time is reached.
  int blocking_until(uint8_t addr, uint8_t* dst, size_t len, bool nostop, absolute_time_t until)
  {
    return i2c_read_blocking_until(i2c_, addr, dst, len, nostop, until);
  }

  //  Attempt to write specified number of bytes to address, with timeout.
  int write_timeout_us(uint8_t addr, const uint8_t* src, size_t len, bool nostop, uint timeout_us)
  {
    return i2c_write_timeout_us(i2c_, addr, src, len, nostop, timeout_us);
  }

  // Attempt to read specified number of bytes from address, with timeout.
  int read_timeout_us(uint8_t addr, uint8_t* dst, size_t len, bool nostop, uint timeout_us)
  {
    return i2c_read_timeout_us(i2c_, addr, dst, len, nostop, timeout_us);
  }

  // Attempt to write specified number of bytes to address, blocking.
  int write_blocking(uint8_t addr, const uint8_t *src, size_t len, bool nostop)
  {
    return i2c_write_blocking(i2c_, addr, src, len, nostop);
  }

  //  Attempt to read specified number of bytes from address, blocking. 
  int read_blocking(uint8_t addr, uint8_t* dst, size_t len, bool nostop)
  {
    return i2c_read_blocking(i2c_, addr, dst, len, nostop);
  }
protected: 
  i2c_inst_t* i2c_;
};

template <typename... T>
class I2CRegister
{
  using AddrT = uint8_t;
  I2CInterface& i2c_;
  uint8_t devAddr_;
  AddrT regAddr_;
  std::vector<uint8_t> buf_;
  bool bufWrittenOnce_;
public:
  I2CRegister(I2CInterface& i2c, uint8_t devAddr, AddrT regAddr) : i2c_{i2c}, devAddr_{devAddr}, regAddr_{regAddr}, bufWrittenOnce_{false}
  {
    buf_.resize((sizeof(AddrT) + ... + sizeof(T)));
    std::memcpy(buf_.data(), (const uint8_t*)(&regAddr_), sizeof(AddrT));
  }
  
  void get(T&... val)
  {
    // Write the address
    i2c_.write_blocking(devAddr_, (const uint8_t*)(&regAddr_), sizeof(AddrT), true);
    // Read in all params
    i2c_.read_blocking(devAddr_, buf_.data() + sizeof(AddrT), (0 + ... + sizeof(T)), false);
    // Unpack the read data from the buffer into the args
    uint8_t* bufData = buf_.data() + sizeof(AddrT);
    ([&]()
    {
      std::memcpy(&val, bufData, sizeof(T));
      bufData += sizeof(T);
    }(), ...);
  }

  void set(T... val)
  {
    // Pack the params into the buffer
    uint8_t* bufData = buf_.data() + sizeof(AddrT);
    ([&]()
    {
      memcpy(bufData, &val, sizeof(T));
      bufData += sizeof(T);
    }(), ...);

    // Write the address + data
    i2c_.write_blocking(devAddr_, buf_.data(), buf_.size(), false);
    bufWrittenOnce_ = true;
  }

  void setIfChanged(T... val)
  {
    if (bufWrittenOnce_)
    {
      // Test if params equal buffer
      uint8_t* bufData = buf_.data() + sizeof(AddrT);
      bool changed = false;
      ([&]()
      {
        if (!changed)
        {
          T valCached;
          memcpy(&valCached, bufData, sizeof(T));
          changed = (val != valCached);
          bufData += sizeof(T);
        }
      }(), ...);
      if (!changed) return;
    }
    set(val...);
  }

};
