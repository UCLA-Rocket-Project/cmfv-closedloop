# Unit Tests

This folder implements the unit tests on both the target embedded system (i.e. the Arduino) and on your native machine (i.e. your laptop or workstation, in case you don't have the appropriate MCU with you on hand). 

We are using [Unity](https://docs.platformio.org/en/stable/advanced/unit-testing/frameworks/unity.html#unit-testing-frameworks-unity) for unit testing, which is a lightweight unit testing framework which can be used for both embedded and native unit testing. 

## Resources

PlatformIO Unit Testing:

- [Unit Testing Docs Homepage](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html)

Unity Testing Framework: 

- [Unity Configuration Guide](https://github.com/ThrowTheSwitch/Unity/blob/master/docs/UnityConfigurationGuide.md)
- [Write your own Custom Test Transport](https://piolabs.com/blog/insights/unit-testing-part-2.htmls)

## Requirements

### Native Unit Tests

You will need to have both gcc and g++ installed on your native device. If you are a Windows user, you can consider trying https://code.visualstudio.com/docs/cpp/config-mingw (I have not tried this before). 

## Writing New Unit Tests

For any directory/sub-directory in `tests/` i.e. this directory that starts with "test", PlatformIO builds the files within the directory as a single build for unit tests. So, for example, PlatformIO will build the `embedded/test_embedded/` and `embedded/test_motor/` directories separately. 

Please place Arduino-only tests into the `embedded/` subfolder, in a proper sub-subfolder (or nest further if you'd like) and desktop-only tests into the `test_desktop/` subfolder.  

Documentation about how specifically to write new unit tests is available online, and you can also see [Resources](#resources). 

### `test_ignore`

The `test_ignore` field allows us to specify which test directories to ignore for a particular env e.g. for the Arduino environment VS the native desktop environment. 

See [this project's platformio.ini file](../platformio.ini) for a concrete usage example. 

## Debugging Unit Tests

### On Arduino

For Arduino UNO R3, PlatformIO on your machine communicates with the Unity-instrumented binary on the MCU via serial. However, if you wish to debug your unit tests running on the Arduino MCU, instead of using something like `Serial.println()`, you should use one of the following (otherwise you'll probably run into issues)

`.pio/build/uno/unity_config/unity_config.cpp`: 

```cpp
void unityOutputStart(unsigned long baudrate) { Serial.begin(baudrate); }
void unityOutputChar(unsigned int c) { Serial.write(c); }
void unityOutputFlush(void) { Serial.flush(); }
void unityOutputComplete(void) { Serial.end(); }
```

Where exactly the default-generated `unity_config.h` and `unity_config.cpp` files exist can be found by running `pio test -vvv` to see the exact compilation command run. 

For more information about `unity_config.h`, see the [Unity Configuration Guide](https://github.com/ThrowTheSwitch/Unity/blob/master/docs/UnityConfigurationGuide.md). 