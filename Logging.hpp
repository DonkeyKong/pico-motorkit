#pragma once

#define LOGGING_ENABLED

#ifdef LOGGING_ENABLED
#include <iostream>
#define DEBUG_LOG(msg) std::cout << msg << std::endl
#define DEBUG_LOG_IF(cond, msg) if(cond) std::cout << msg << std::endl
#else
#define DEBUG_LOG(msg) do { } while(0)
#define DEBUG_LOG_IF(cond, msg) do { } while(0)
#endif