#pragma once

#include <string>
#include "GPSsample.hpp"

// Keep these macros consistent with receiver.cpp / IMUserver.cpp usage.
#ifndef MAX
#define MAX 1024
#endif

#ifndef PORT
#define PORT 7777
#endif

#ifndef SA
#define SA struct sockaddr
#endif

namespace GPS{
    bool parse_GPS(const std::string& line, GPSsample& out);
    void process(int connfd);
}