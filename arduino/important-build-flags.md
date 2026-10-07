# Important Build Flags

## In [include/config.h](./include/config.h)

### `IS_FUEL_SYSTEM`

Activates different constants for OCV / FCV based on whether it is true

### `OPEN_LOOP_MODE`

Dictates whether there will be any closed loop control. If true, then control will entirely be open loop i.e. we go into `FORCED_OPEN_LOOP` right after `OPEN_LOOP_INIT`

### `USE_3_PTS`

If true, code will compile that uses a 3-PT (PT stands for pressure transducer) redundant system to obtain manifold pressures as input into the control system, instead of the 2-PT system. 

With the 3-PT system, the median PT value will be taken (if all the PT values are within P_MAX and P_MIN). 

## In [platformio.ini](./platformio.ini)

Please note that build flags have to be applied separately to the [env:uno] and [env:native] environments. Applying a build flag to one does not result in it being applied to the other. 

### `BUILD_ARDUINO` and `BUILD_NATIVE`

If defined, simply means that a particular build is for either Arduino or Native (your workstation)

### `NO_MANUAL_ABORT`

Prevents compilation of code that checks for manual abort - useful if there is no manual abort pin connected on the Arduino

### `USE_OSCILLATION_DETECTOR`

Use a specific methodology defined in [controller.cpp](./lib/modules/src/controller.cpp) to detect oscillations

> NOTE: This method of FDIR (Fault Detection, Isolation and Recovery) has a relatively complex trigger condition and is thus discouraged. 