#ifndef UTILITIES_H
#define UTILITIES_H

#include <AMT22_lib.h>

#include "state_machine.h"
#include <comm_handler.h>
#include <controller.h>
#include <pressure_sensor.h>

// External hardware objects (declared in main.cpp)
extern AMT22* encoder;
extern Controller* controller;
extern PressureSensor* pressureSensor;
extern CommHandler* commHandler;
extern ChannelState channel;
extern FaultFlags faults;
extern SystemState systemState;

// Encoder utility functions
float getEncoderAngle();
bool isAngleValid(float angle);
bool isEncoderHealthy();

// System utility functions
bool readManifoldPressures(float& pressure);
void setMPV(bool open); // This sets whether we should deactuate (close) MPV, as a signal sent to the Pi. We do not directly control MPV
void setMPVState(bool state); // This sets whether we have detected MPV
bool getMPVState();
bool isManualAbortPressed();
void applyMoveFilter(float& deltaAngle);
float constrainAngle(float angle);
void publishTelemetry();
void resetSystemOnMpvCycle();
bool isValidState(SystemStateEnum s);
bool isAtStartAngle(float currentAngle);
bool checkRedBand(float pressure);
SystemStateEnum getSyncedState(SystemStateEnum currentState, SystemStateEnum otherControllerState, float currentAngle);

#endif // UTILITIES_H