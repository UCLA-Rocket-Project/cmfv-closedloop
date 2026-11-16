#pragma once

// Text-only diagnostic build. AVR printf normally does not support %f;
// all angles are emitted as signed integer hundredths of a degree.
#ifdef MOTOR_DEBUG_TEXT
#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include <avr/pgmspace.h>
#define MOTOR_LOG(fmt, ...) printf_P(PSTR(fmt), ##__VA_ARGS__)
static inline long debugCentiDeg(float angle) { return lroundf(angle * 100.0f); }
#else
#define MOTOR_LOG(...) do {} while (0)
#endif
