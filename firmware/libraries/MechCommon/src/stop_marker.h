#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// newlib-nano on UNO R4 does not support printf's long-long conversion.
static inline int encodeEnded(char *output, size_t size, const char *id,
                       int64_t sequence, const char *reason, uint32_t dropped) {
  char sequence_text[12];
  if (sequence < 0) strcpy(sequence_text, "-1");
  else snprintf(sequence_text, sizeof sequence_text, "%lu", (unsigned long)sequence);
  return snprintf(output, size, "# ENDED,3,%s,%s,%s,%lu\n", id, sequence_text,
                  reason, (unsigned long)dropped);
}
