#ifndef ACADOS_SOLVERS__VISIBILITY_CONTROL_H_
#define ACADOS_SOLVERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACADOS_SOLVERS_EXPORT __attribute__ ((dllexport))
    #define ACADOS_SOLVERS_IMPORT __attribute__ ((dllimport))
  #else
    #define ACADOS_SOLVERS_EXPORT __declspec(dllexport)
    #define ACADOS_SOLVERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACADOS_SOLVERS_BUILDING_LIBRARY
    #define ACADOS_SOLVERS_PUBLIC ACADOS_SOLVERS_EXPORT
  #else
    #define ACADOS_SOLVERS_PUBLIC ACADOS_SOLVERS_IMPORT
  #endif
  #define ACADOS_SOLVERS_PUBLIC_TYPE ACADOS_SOLVERS_PUBLIC
  #define ACADOS_SOLVERS_LOCAL
#else
  #define ACADOS_SOLVERS_EXPORT __attribute__ ((visibility("default")))
  #define ACADOS_SOLVERS_IMPORT
  #if __GNUC__ >= 4
    #define ACADOS_SOLVERS_PUBLIC __attribute__ ((visibility("default")))
    #define ACADOS_SOLVERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACADOS_SOLVERS_PUBLIC
    #define ACADOS_SOLVERS_LOCAL
  #endif
  #define ACADOS_SOLVERS_PUBLIC_TYPE
#endif

#endif  // ACADOS_SOLVERS__VISIBILITY_CONTROL_H_
