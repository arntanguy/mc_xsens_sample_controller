#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define XsensSampleController_DLLIMPORT __declspec(dllimport)
#  define XsensSampleController_DLLEXPORT __declspec(dllexport)
#  define XsensSampleController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define XsensSampleController_DLLIMPORT __attribute__((visibility("default")))
#    define XsensSampleController_DLLEXPORT __attribute__((visibility("default")))
#    define XsensSampleController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define XsensSampleController_DLLIMPORT
#    define XsensSampleController_DLLEXPORT
#    define XsensSampleController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef XsensSampleController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define XsensSampleController_DLLAPI
#  define XsensSampleController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef XsensSampleController_EXPORTS
#    define XsensSampleController_DLLAPI XsensSampleController_DLLEXPORT
#  else
#    define XsensSampleController_DLLAPI XsensSampleController_DLLIMPORT
#  endif // XsensSampleController_EXPORTS
#  define XsensSampleController_LOCAL XsensSampleController_DLLLOCAL
#endif // XsensSampleController_STATIC