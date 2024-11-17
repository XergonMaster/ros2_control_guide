#ifndef GRIPPER_CTRL_PKG__VISIBILITY_CONTROL_H_
#define GRIPPER_CTRL_PKG__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GRIPPER_CTRL_PKG_EXPORT __attribute__((dllexport))
#define GRIPPER_CTRL_PKG_IMPORT __attribute__((dllimport))
#else
#define GRIPPER_CTRL_PKG_EXPORT __declspec(dllexport)
#define GRIPPER_CTRL_PKG_IMPORT __declspec(dllimport)
#endif
#ifdef GRIPPER_CTRL_PKG_BUILDING_LIBRARY
#define GRIPPER_CTRL_PKG_PUBLIC GRIPPER_CTRL_PKG_EXPORT
#else
#define GRIPPER_CTRL_PKG_PUBLIC GRIPPER_CTRL_PKG_IMPORT
#endif
#define GRIPPER_CTRL_PKG_PUBLIC_TYPE GRIPPER_CTRL_PKG_PUBLIC
#define GRIPPER_CTRL_PKG_LOCAL
#else
#define GRIPPER_CTRL_PKG_EXPORT __attribute__((visibility("default")))
#define GRIPPER_CTRL_PKG_IMPORT
#if __GNUC__ >= 4
#define GRIPPER_CTRL_PKG_PUBLIC __attribute__((visibility("default")))
#define GRIPPER_CTRL_PKG_LOCAL __attribute__((visibility("hidden")))
#else
#define GRIPPER_CTRL_PKG_PUBLIC
#define GRIPPER_CTRL_PKG_LOCAL
#endif
#define GRIPPER_CTRL_PKG_PUBLIC_TYPE
#endif

#endif // GRIPPER_CTRL_PKG__VISIBILITY_CONTROL_H_
