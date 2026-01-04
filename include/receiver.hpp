#pragma once

#include <string>
#include "IMUsample.hpp"

// Keep these macros consistent with receiver.cpp / IMUserver.cpp usage.
#ifndef MAX
#define MAX 1024
#endif

#ifndef PORT
#define PORT 8888
#endif

#ifndef SA
#define SA struct sockaddr
#endif

// Same signature as in receiver.cpp
bool parse_one_quat_accg(const std::string& line, IMUsample& out);

// Same signature as in receiver.cpp
void process(int connfd);
