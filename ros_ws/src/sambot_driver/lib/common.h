#pragma once

#if defined(_WIN32)
#include "impl\windows\win.h"
#include "impl\windows\win_serial.h"
#elif defined(__GNUC__)
#include "impl/unix/unix.h"
#include "impl/unix/unix_serial.h"
#else
#error "unsupported target"
#endif
#include "ydlidar/locker.h"
#include "ydlidar/thread.h"
#include "ydlidar/timer.h"

#define SDKVerision "1.4.2"
