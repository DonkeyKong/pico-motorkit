#pragma once

#define LOGGING_ENABLED

#ifdef LOGGING_ENABLED
#include <iostream>
#define DEBUG_LOG(msg) std::cout << msg << std::endl
#else
do { } while(0)
#endif