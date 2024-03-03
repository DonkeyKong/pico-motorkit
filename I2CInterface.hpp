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
  int read_blocking_until(uint8_t addr, uint8_t* dst, size_t len, bool nostop, absolute_time_t until)
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

  // Attempt to write to the device non-blocking
  // bool write_async(uint8_t addr, const uint8_t *src, size_t len, bool nostop)
  // {
  //   i2c_->hw->enable = 0;
  //   i2c_->hw->tar = addr;
  //   i2c_->hw->enable = 1;

  //   // bool abort = false;
  //   // bool timeout = false;
  //   //uint32_t abort_reason = 0;

  //   int byte_ctr;
  //   int ilen = (int)len;
  //   for (byte_ctr = 0; byte_ctr < ilen; ++byte_ctr) 
  //   {
  //       bool first = byte_ctr == 0;
  //       bool last = byte_ctr == ilen - 1;

  //       i2c_->hw->data_cmd =
  //               bool_to_bit(first && i2c_->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
  //               bool_to_bit(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB |
  //               *src++;

  //       // // Wait until the transmission of the address/data from the internal
  //       // // shift register has completed. For this to function correctly, the
  //       // // TX_EMPTY_CTRL flag in IC_CON must be set. The TX_EMPTY_CTRL flag
  //       // // was set in i2c_init.
  //       // do {
  //       //     if (timeout_check) {
  //       //         timeout = timeout_check(ts);
  //       //         abort |= timeout;
  //       //     }
  //       //     tight_loop_contents();
  //       // } while (!timeout && !(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS));

  //       // If there was a timeout, don't attempt to do anything else.
  //       if (!timeout) 
  //       {
  //           abort_reason = i2c->hw->tx_abrt_source;
  //           if (abort_reason) {
  //               // Note clearing the abort flag also clears the reason, and
  //               // this instance of flag is clear-on-read! Note also the
  //               // IC_CLR_TX_ABRT register always reads as 0.
  //               i2c->hw->clr_tx_abrt;
  //               abort = true;
  //           }

  //           if (abort || (last && !nostop)) {
  //               // If the transaction was aborted or if it completed
  //               // successfully wait until the STOP condition has occured.

  //               // TODO Could there be an abort while waiting for the STOP
  //               // condition here? If so, additional code would be needed here
  //               // to take care of the abort.
  //               do {
  //                   if (timeout_check) {
  //                       timeout = timeout_check(ts);
  //                       abort |= timeout;
  //                   }
  //                   tight_loop_contents();
  //               } while (!timeout && !(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS));

  //               // If there was a timeout, don't attempt to do anything else.
  //               if (!timeout) {
  //                   i2c->hw->clr_stop_det;
  //               }
  //           }
  //       }

  //       // Note the hardware issues a STOP automatically on an abort condition.
  //       // Note also the hardware clears RX FIFO as well as TX on abort,
  //       // because we set hwparam IC_AVOID_RX_FIFO_FLUSH_ON_TX_ABRT to 0.
  //       if (abort)
  //           break;
  //   }

  //   int rval;

  //   // A lot of things could have just happened due to the ingenious and
  //   // creative design of I2C. Try to figure things out.
  //   // if (abort) {
  //   //     if (timeout)
  //   //         rval = PICO_ERROR_TIMEOUT;
  //   //     else if (!abort_reason || abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS) {
  //   //         // No reported errors - seems to happen if there is nothing connected to the bus.
  //   //         // Address byte not acknowledged
  //   //         rval = PICO_ERROR_GENERIC;
  //   //     } else if (abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS) {
  //   //         // Address acknowledged, some data not acknowledged
  //   //         rval = byte_ctr;
  //   //     } else {
  //   //         //panic("Unknown abort from I2C instance @%08x: %08x\n", (uint32_t) i2c->hw, abort_reason);
  //   //         rval = PICO_ERROR_GENERIC;
  //   //     }
  //   // } else {
  //        rval = byte_ctr;
  //   // }

  //   // nostop means we are now at the end of a *message* but not the end of a *transfer*
  //   i2c_->restart_on_next = nostop;
  //   return rval;
  // }
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