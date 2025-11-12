// This file defines ASSERT_OWN_H and is called assert_own.h so it does not conflict with
// assert.h files of existing frameworks (and so the include guards do not conflict too). 

#ifndef ASSERT_OWN_H
#define ASSERT_OWN_H

#define QUOTE_detail(arg) #arg
#define QUOTE(arg) QUOTE_detail(arg)

#ifndef PIO_UNIT_TESTING /* Production build, don't implement asserts */
#   define assert(arg) /* Nothing */
#elif defined(BUILD_ARDUINO)
#   include <Arduino.h>
#   define assert(arg)                                                                    \
        do                                                                                \
        {                                                                                 \
            if ( false == static_cast<bool>(arg) )                                        \
            {                                                                             \
                Serial.println("assert failed on line #" QUOTE(__LINE__) " : '" #arg "'"  \
                            " -- now entering infinite loop");                            \
                for (;;) yield();                                                         \
            }                                                                             \
        } while ( false )
#elif defined(BUILD_NATIVE) 
#   include <cassert> /* assert() is alr defined */
#else
#   error "Either one of BUILD_NATIVE or BUILD_ARDUINO should be set"
#endif

#endif // ASSERT_OWN_H