#ifndef ACADOS_SOLVER_PLUGINS__VISIBILITY_CONTROL_H_
#define ACADOS_SOLVER_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACADOS_SOLVER_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define ACADOS_SOLVER_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define ACADOS_SOLVER_PLUGINS_EXPORT __declspec(dllexport)
    #define ACADOS_SOLVER_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACADOS_SOLVER_PLUGINS_BUILDING_LIBRARY
    #define ACADOS_SOLVER_PLUGINS_PUBLIC ACADOS_SOLVER_PLUGINS_EXPORT
  #else
    #define ACADOS_SOLVER_PLUGINS_PUBLIC ACADOS_SOLVER_PLUGINS_IMPORT
  #endif
  #define ACADOS_SOLVER_PLUGINS_PUBLIC_TYPE ACADOS_SOLVER_PLUGINS_PUBLIC
  #define ACADOS_SOLVER_PLUGINS_LOCAL
#else
  #define ACADOS_SOLVER_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define ACADOS_SOLVER_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define ACADOS_SOLVER_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define ACADOS_SOLVER_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACADOS_SOLVER_PLUGINS_PUBLIC
    #define ACADOS_SOLVER_PLUGINS_LOCAL
  #endif
  #define ACADOS_SOLVER_PLUGINS_PUBLIC_TYPE
#endif

#endif  // ACADOS_SOLVER_PLUGINS__VISIBILITY_CONTROL_H_
