#ifndef QUADROTOR_CONTROL__VISIBILITY_CONTROL_H_
#define QUADROTOR_CONTROL__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define QUADROTOR_CONTROL_EXPORT __attribute__ ((dllexport))
    #define QUADROTOR_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define QUADROTOR_CONTROL_EXPORT __declspec(dllexport)
    #define QUADROTOR_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef QUADROTOR_CONTROL_BUILDING_DLL
    #define QUADROTOR_CONTROL_PUBLIC QUADROTOR_CONTROL_EXPORT
  #else
    #define QUADROTOR_CONTROL_PUBLIC QUADROTOR_CONTROL_IMPORT
  #endif
  #define QUADROTOR_CONTROL_PUBLIC_TYPE QUADROTOR_CONTROL_PUBLIC
  #define QUADROTOR_CONTROL_LOCAL
#else
  #define QUADROTOR_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define QUADROTOR_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define QUADROTOR_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define QUADROTOR_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define QUADROTOR_CONTROL_PUBLIC
    #define QUADROTOR_CONTROL_LOCAL
  #endif
  #define QUADROTOR_CONTROL_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_CONTROL__VISIBILITY_CONTROL_H_