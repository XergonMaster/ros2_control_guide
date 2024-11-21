#ifndef YB_EB__VISIBILITY_CONTROL_H_
#define YB_EB__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define YB_EB_EXPORT __attribute__((dllexport))
#define YB_EB_IMPORT __attribute__((dllimport))
#else
#define YB_EB_EXPORT __declspec(dllexport)
#define YB_EB_IMPORT __declspec(dllimport)
#endif
#ifdef YB_EB_BUILDING_LIBRARY
#define YB_EB_PUBLIC YB_EB_EXPORT
#else
#define YB_EB_PUBLIC YB_EB_IMPORT
#endif
#define YB_EB_PUBLIC_TYPE YB_EB_PUBLIC
#define YB_EB_LOCAL
#else
#define YB_EB_EXPORT __attribute__((visibility("default")))
#define YB_EB_IMPORT
#if __GNUC__ >= 4
#define YB_EB_PUBLIC __attribute__((visibility("default")))
#define YB_EB_LOCAL __attribute__((visibility("hidden")))
#else
#define YB_EB_PUBLIC
#define YB_EB_LOCAL
#endif
#define YB_EB_PUBLIC_TYPE
#endif

#endif // YB_EB__VISIBILITY_CONTROL_H_
