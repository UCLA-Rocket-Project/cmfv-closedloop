# Important Build Flags

## `IS_FUEL_SYSTEM`

- What it does: Activates different constants for OCV / FCV based on whether it is true
- Where to edit it: [include/config.h](./include/config.h)

## `BUILD_ARDUINO` and `BUILD_NATIVE`

- What they do: If defined, simply mean that a particular build is for either Arduino or Native
- Where to edit it: [platformio.ini](./platformio.ini)

## `NO_MANUAL_ABORT`

- What it does: Prevents compilation of code that checks for manual abort - useful if there is no manual abort pin connected on the Arduino
- Where to edit it: [platformio.ini](./platformio.ini)

## `USE_OSCILLATION_DETECTOR`

- What it does: Use a specific methodology defined in [controller.cpp](./lib/modules/src/controller.cpp) to detect oscillations
- Where to add it: [platformio.ini](./platformio.ini)