#include "Button.hpp"
#include "DiscreteOut.hpp"
#include "Settings.hpp"
#include "I2CInterface.hpp"
#include "MotorKit.hpp"
#include "Logging.hpp"

#include <hardware/watchdog.h>
#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <pico/bootrom.h>

#include <memory>
#include <iostream>
#include <istream>
#include <sstream>
#include <cstring>
#include <string>

using namespace std::chrono_literals;

//GPIOButton button(3);
//DiscreteOut outputLine(4);
std::unique_ptr<MotorKit> motors;

void releaseSteppers()
{
  motors->stepper0().release();
  motors->stepper1().release();
}

void rebootIntoProgMode()
{
  releaseSteppers();
  //multicore_reset_core1();  // Only needed for multicore
  reset_usb_boot(0,0);
}

template <typename T>
bool setValFromStream(T& val, T min, T max, std::istream& s)
{
  T input;
  s >> input;
  if (s.fail())
  {
    std::cout << "parse error" << std::endl << std::flush;
    return false;
  }
  if (input < min || input > max)
  {
    std::cout << "value out of range error" << std::endl << std::flush;
    return false;
  }
  val = input;
  return true;
}

bool setValFromStream(bool& val, std::istream& s)
{
  int input;
  s >> input;
  if (s.fail())
  {
    std::cout << "parse error" << std::endl << std::flush;
    return false;
  }
  if (input < 0 || input > 1)
  {
    std::cout << "value out of range error" << std::endl << std::flush;
    return false;
  }
  val = (input == 1);
  return true;
}

bool setValFromStream(char* val, size_t len, std::istream& ss)
{
  // Eat any extra whitespace in the stream
  ss >> std::ws;
  // Get the whole rest of the line as a string
  std::string str;
  std::getline(ss, str);

  // Check if the read was clean and if the string will fit
  if (ss.fail())
  {
    std::cout << "parse error" << std::endl << std::flush;
    return false;
  }
  if (str.size() >= len)
  {
    std::cout << "string param too long" << std::endl << std::flush;
    return false;
  }
  // Copy the string to the dest buffer will null terminator
  memcpy(val, str.data(), str.size());
  val[str.size()] = '\0';
  return true;
}

void processCommand(std::string cmdAndArgs, Settings& settings)
{
  std::stringstream ss(cmdAndArgs);
  std::string cmd;
  ss >> cmd;
  
  if (cmd == "defaults")
  {
    settings.setDefaults();
  }
  else if (cmd == "flash")
  {
    // Write the settings to flash
    if (settings.writeToFlash())
      std::cout << "Wrote settings to flash!" << std::endl << std::flush;
    else
      std::cout << "Skipped writing to flash because contents were already correct." << std::endl << std::flush;
  }
  else if (cmd == "info" || cmd == "about")
  {
    std::cout << "CoreXY by Donkey Kong" << std::endl;
    std::cout << std::endl;
    settings.print();
    std::cout << std::endl;
    std::cout << "-- Runtime Data --" << std::endl;
    std::cout << "full settings size: " << sizeof(Settings) << std::endl;
    std::cout << std::flush;
  }
  else if (cmd == "reboot")
  {
    // Reboot the system immediately
    std::cout << "ok" << std::endl << std::flush;
    releaseSteppers();
    watchdog_reboot(0,0,0);
  }
  else if (cmd == "prog")
  {
    // Reboot into programming mode
    std::cout << "ok" << std::endl << std::flush;
    rebootIntoProgMode();
  }
  else if (cmd == "move")
  {
    int stepper, steps;
    if (setValFromStream(stepper, 0, 1, ss) &&
        setValFromStream(steps, -200 * 16 * 10, 200 * 16 * 10, ss))
    {
      motors->getStepper(stepper).move(steps, 30.0f);
    }
  }
  else if (cmd == "movedeg")
  {
    int stepper;
    float deg;
    if (setValFromStream(stepper, 0, 1, ss) &&
        setValFromStream(deg, -360.0f * 10.0f, 360.0f * 10.0f, ss))
    {
      motors->getStepper(stepper).moveDegrees(deg, 50.0f);
    }
  }
  else if (cmd == "power")
  {
    int stepper;
    float power;
    if (setValFromStream(stepper, 0, 1, ss) &&
        setValFromStream(power, 0.0f, 1.0f, ss))
    {
      motors->getStepper(stepper).setPower(power);
    }
  }
  else if (cmd == "mode")
  {
    int stepper, mode;
    if (setValFromStream(stepper, 0, 1, ss) &&
        setValFromStream(mode, 0, 2, ss))
    {
      motors->getStepper(stepper).setMode((Stepper::Mode)mode);
    }
  }
  else if (cmd == "release")
  {
    int stepper;
    if (setValFromStream(stepper, 0, 1, ss))
    {
      motors->getStepper(stepper).release();
    }
  }
  else
  {
    std::cout << "unknown command error" << std::endl << std::flush;
    return;
  }

  if (!ss.fail())
  {
    std::cout << "ok" << std::endl << std::flush;
  }
}

void processStdIo(Settings& settings)
{
  static char inBuf[1024];
  static int pos = 0;

  while (true)
  {
    int inchar = getchar_timeout_us(0);
    if (inchar > 31 && inchar < 127 && pos < 1023)
    {
      inBuf[pos++] = (char)inchar;
      std::cout << (char)inchar << std::flush; // echo to client
    }
    else if (inchar == '\n')
    {
      inBuf[pos] = '\0';
      std::cout << std::endl << std::flush; // echo to client
      processCommand(inBuf, settings);
      pos = 0;
    }
    else
    {
      return;
    }
  }
}

int main()
{
  // Configure stdio
  stdio_init_all();

  DEBUG_LOG("Initialized stdio");

  // Wait 1 second for remote terminals to connect
  // before doing anything.
  sleep_ms(1000);
  DEBUG_LOG("Slept 1000ms to wait for console attach");

  // Init the settings object
  SettingsManager settingsMgr;
  Settings& settings = settingsMgr.getSettings();

  // Init the i2c bus and steppers
  I2CInterface i2c(i2c0, 0, 1, 400000);
  motors = std::make_unique<MotorKit>(i2c);
  
  absolute_time_t nextFrameTime = get_absolute_time();

  while (1)
  {
    // Regulate loop speed
    sleep_until(nextFrameTime);
    nextFrameTime = make_timeout_time_ms(50);

    // Process input
    processStdIo(settings);
  }
  return 0;
}
