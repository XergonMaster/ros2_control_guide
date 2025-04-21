#ifndef ARDUINO_INTERFACE__VISIBILITY_CONTROL_H_
#define ARDUINO_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ARDUINO_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define ARDUINO_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define ARDUINO_INTERFACE_EXPORT __declspec(dllexport)
    #define ARDUINO_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ARDUINO_INTERFACE_BUILDING_LIBRARY
    #define ARDUINO_INTERFACE_PUBLIC ARDUINO_INTERFACE_EXPORT
  #else
    #define ARDUINO_INTERFACE_PUBLIC ARDUINO_INTERFACE_IMPORT
  #endif
  #define ARDUINO_INTERFACE_PUBLIC_TYPE ARDUINO_INTERFACE_PUBLIC
  #define ARDUINO_INTERFACE_LOCAL
#else
  #define ARDUINO_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define ARDUINO_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define ARDUINO_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define ARDUINO_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ARDUINO_INTERFACE_PUBLIC
    #define ARDUINO_INTERFACE_LOCAL
  #endif
  #define ARDUINO_INTERFACE_PUBLIC_TYPE
#endif

#endif  // ARDUINO_INTERFACE__VISIBILITY_CONTROL_H_
