#ifndef ROBOT_HARDWARE_BRIDGE__VISIBILITY_CONTROL_H_
#define ROBOT_HARDWARE_BRIDGE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_HARDWARE_BRIDGE_EXPORT __attribute__ ((dllexport))
    #define ROBOT_HARDWARE_BRIDGE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_HARDWARE_BRIDGE_EXPORT __declspec(dllexport)
    #define ROBOT_HARDWARE_BRIDGE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_HARDWARE_BRIDGE_BUILDING_LIBRARY
    #define ROBOT_HARDWARE_BRIDGE_PUBLIC ROBOT_HARDWARE_BRIDGE_EXPORT
  #else
    #define ROBOT_HARDWARE_BRIDGE_PUBLIC ROBOT_HARDWARE_BRIDGE_IMPORT
  #endif
  #define ROBOT_HARDWARE_BRIDGE_PUBLIC_TYPE ROBOT_HARDWARE_BRIDGE_PUBLIC
  #define ROBOT_HARDWARE_BRIDGE_LOCAL
#else
  #define ROBOT_HARDWARE_BRIDGE_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_HARDWARE_BRIDGE_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_HARDWARE_BRIDGE_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_HARDWARE_BRIDGE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_HARDWARE_BRIDGE_PUBLIC
    #define ROBOT_HARDWARE_BRIDGE_LOCAL
  #endif
  #define ROBOT_HARDWARE_BRIDGE_PUBLIC_TYPE
#endif

#endif  // ROBOT_HARDWARE_BRIDGE__VISIBILITY_CONTROL_H_
