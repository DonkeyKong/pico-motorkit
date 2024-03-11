#include "Button.hpp"
#include "DiscreteOut.hpp"
#include "Settings.hpp"
#include "I2CInterface.hpp"
#include "MotorKit.hpp"
#include "MotionXY.hpp"
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

GPIOButton limitSwitchX(2);
GPIOButton limitSwitchY(3);
//DiscreteOut outputLine(4);
std::unique_ptr<MotorKit> motors;
std::unique_ptr<Stage2D> stage;
float moveRpm = 120.0f;
double moveMmPerSec = 40.0;

template <typename T>
inline bool between(T val, T bound1, T bound2)
{
  if (bound1 > bound2) return val >= bound2 && val <= bound1;
  return val >= bound1 && val <= bound2;
}

void releaseSteppers()
{
  if (motors->getStepper(0)) motors->getStepper(0)->release();
  if (motors->getStepper(1)) motors->getStepper(1)->release();
}

void rebootIntoProgMode()
{
  releaseSteppers();
  //multicore_reset_core1();  // Only needed for multicore
  reset_usb_boot(0,0);
}

void circleMove()
{
  double centerX = stage->sizeX / 2.0;
  double centerY = stage->sizeY / 2.0;
  double radius = std::min(centerX, centerY);
  for (double theta = 0; theta < 6.28; theta += 0.1)
  {
    stage->moveTo(radius * std::cos(theta) + centerX, radius * -std::sin(theta) + centerY, moveMmPerSec);
  }
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
  if (!between(input, min, max))
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
  else if (cmd == "stepper")
  {
    int stepper;
    if (setValFromStream(stepper, 0, 1, ss))
    {
      ss >> cmd;
      if (cmd == "move")
      {
        int steps;
        if (setValFromStream(steps, -200 * 16 * 10, 200 * 16 * 10, ss))
        {
          if (motors->getStepper(stepper)) motors->getStepper(stepper)->move(steps, moveRpm);
        }
      }
      else if (cmd == "rpm")
      {
        setValFromStream(moveRpm, 0.1f, 3000.0f, ss);
      }
      else if (cmd == "movedeg")
      {
        float deg;
        if (setValFromStream(deg, -360.0f * 10.0f, 360.0f * 10.0f, ss))
        {
          if (motors->getStepper(stepper)) motors->getStepper(stepper)->moveDegrees(deg, moveRpm);
        }
      }
      else if (cmd == "power")
      {
        float power;
        if (setValFromStream(power, 0.0f, 1.0f, ss))
        {
          if (motors->getStepper(stepper)) motors->getStepper(stepper)->setPower(power);
        }
      }
      else if (cmd == "mode")
      {
        int mode;
        if (setValFromStream(mode, 0, 2, ss))
        {
          if (motors->getStepper(stepper)) motors->getStepper(stepper)->setMode((Stepper::Mode)mode);
        }
      }
      else if (cmd == "release")
      {
        if (motors->getStepper(stepper)) motors->getStepper(stepper)->release();
      }
      else
      {
        std::cout << "unknown stepper command error" << std::endl << std::flush;
        return;
      }
    }
  }
  else if (cmd == "stage")
  {
    ss >> cmd;
    if (cmd == "home")
    {
      std::cout << "Homing the stage..." << std::endl;
      bool homed = stage->home();
      if (!homed) std::cout << "Homing failed!" << std::endl;
    }
    else if (cmd == "fhome")
    {
      int64_t u, v;
      if (setValFromStream(u, 0ll, INT64_MAX, ss) &&
          setValFromStream(v, 0ll, INT64_MAX, ss))
      {
        stage->forceHomed(u, v);
      }
    }
    else if (cmd == "speed")
    {
      setValFromStream(moveMmPerSec, 0.1, 1000.0, ss);
    }
    else if (cmd == "move")
    {
      double x, y;
      if (setValFromStream(x, 0.0, stage->sizeX, ss) &&
          setValFromStream(y, 0.0, stage->sizeY, ss))
      {
        stage->stop();
        stage->moveTo(x, y, moveMmPerSec);
        stage->completeAllMoves();
      }
    }
    else if (cmd == "circle")
    {
      int count;
      if (setValFromStream(count, 1, 60, ss))
      {
        stage->stop();
        for (int i = 0; i < count; ++i) circleMove();
        stage->completeAllMoves();
      }
    }
    else if (cmd == "rel")
    {
      double x, y;
      if (setValFromStream(x, -stage->sizeX, stage->sizeX, ss) &&
          setValFromStream(y, -stage->sizeY, stage->sizeY, ss))
      {
        stage->stop();
        stage->moveRel(x, y, moveMmPerSec);
        stage->completeAllMoves();
      }
    }
    else if (cmd == "qmove")
    {
      double x, y;
      if (setValFromStream(x, 0.0, stage->sizeX, ss) &&
          setValFromStream(y, 0.0, stage->sizeY, ss))
      {
        stage->moveTo(x, y, moveMmPerSec);
      }
    }
    else if (cmd == "qrel")
    {
      double x, y;
      if (setValFromStream(x, -stage->sizeX, stage->sizeX, ss) &&
          setValFromStream(y, -stage->sizeY, stage->sizeY, ss))
      {
        stage->moveRel(x, y, moveMmPerSec);
      }
    }
    else if (cmd == "go")
    {
      stage->completeAllMoves();
    }
    else
    {
      std::cout << "unknown stage command error" << std::endl << std::flush;
      return;
    }
  }
  else if (cmd == "releaseall")
  {
    releaseSteppers();
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
  static std::string lastCmd;

  while (true)
  {
    int inchar = getchar_timeout_us(0);
    if (inchar > 31 && inchar < 127 && pos < 1023)
    {
      inBuf[pos++] = (char)inchar;
      std::cout << (char)inchar << std::flush; // echo to client
    }
    else if (inchar == '\b' && pos > 0) // handle backspaces
    {
      --pos;
      std::cout << "\b \b" << std::flush;
    }
    else if (inchar == '\t' && lastCmd.size() < 1023) // handle tab to insert last command
    {
      while (pos-- > 0) std::cout << "\b \b" << std::flush;
      memcpy(inBuf, lastCmd.data(), lastCmd.size());
      pos = lastCmd.size();
      std::cout << lastCmd << std::flush;
    }
    else if (inchar == '\n')
    {
      inBuf[pos] = '\0';
      std::cout << std::endl << std::flush; // echo to client
      lastCmd = inBuf;
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

  #ifdef LOGGING_ENABLED
  sleep_ms(2000);
  #endif

  // Init the settings object
  SettingsManager settingsMgr;
  Settings& settings = settingsMgr.getSettings();

  // Init the i2c bus and steppers
  // The docs for the motorkit hat say it supports up to
  // 1 MHz communication rate
  I2CInterface i2c(i2c0, 0, 1, 1000000);
  motors = std::make_unique<MotorKit>(i2c);

  // Connect some NEMA-17 steppers with:
  //    200 steps per rev
  //    8 microsteps
  //    230 rpm max speed
  motors->connectStepper(0, {200, 8, 230, true});
  motors->connectStepper(1, {200, 8, 230, true});

  motors->getStepper(0)->setMode(Stepper::Mode::Microstep);
  motors->getStepper(1)->setMode(Stepper::Mode::Microstep);
  
  stage = std::make_unique<StageCoreXY>(
    *motors->getStepper(0), *motors->getStepper(1), 
    limitSwitchX, limitSwitchY, 
    75.0, 340.0, 
    2.0, 3.0, 25.0,
    // GT2 pulleys seem to be 40 mm per revolution
    (double)motors->getStepper(0)->motorInfo.StepsPerRev / 40.0, 
    (double)motors->getStepper(1)->motorInfo.StepsPerRev / 40.0
  );

  // Set the stage to release steppers after completing a move sequence
  stage->setMode(Stage2D::StageMoveMode::Release);

  DEBUG_LOG("Created stage of size ( " << stage->sizeX << " , " << stage->sizeY << " ) mm");

  absolute_time_t nextFrameTime = get_absolute_time();

  while (1)
  {
    // Regulate loop speed
    sleep_until(nextFrameTime);
    nextFrameTime = make_timeout_time_ms(50);

    limitSwitchX.update();
    limitSwitchY.update();

    // Debug for limit switches
    DEBUG_LOG_IF(limitSwitchX.buttonDown(), "Limit X pressed");
    DEBUG_LOG_IF(limitSwitchX.buttonUp(), "Limit X released");
    DEBUG_LOG_IF(limitSwitchY.buttonDown(), "Limit Y pressed");
    DEBUG_LOG_IF(limitSwitchY.buttonUp(), "Limit Y released");

    // Process input
    processStdIo(settings);
  }
  return 0;
}
