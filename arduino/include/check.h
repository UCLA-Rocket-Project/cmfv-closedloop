#ifndef CHECK
#define CHECK

#include <stddef.h>
#include "config.h"
#include "comm_handler.h"

static_assert(sizeof(SystemStateEnum) == 1,
              "SystemStateEnum must be 1 byte");

static_assert(sizeof(telemetryPacket_t) == (USE_3_PTS ? 36 : 32),
              "Telemetry packet size mismatch");

static_assert(sizeof(pressureUpdatePacket_t) == (USE_3_PTS ? 24 : 20),
              "Pressure update packet size mismatch");

static_assert(offsetof(telemetryPacket_t, packetNo) ==
              (USE_3_PTS ? 32 : 28),
              "Telemetry packetNo offset mismatch");

static_assert(offsetof(pressureUpdatePacket_t, packetNo) ==
              (USE_3_PTS ? 20 : 16),
              "Pressure update packetNo offset mismatch");
#endif