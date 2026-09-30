#pragma once
#include <stddef.h>
#include "scan_snapshot.h"
struct TelemetryConfig { int centre_dac; int half_range; };
int encodeTelemetry(const ScanSnapshot &sample, const TelemetryConfig &config, char *buffer, size_t capacity);
