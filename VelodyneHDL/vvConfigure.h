#ifndef __vvConfigure_h
#define __vvConfigure_h

#if defined(WIN32)

 #if defined(VelodyneHDLPlugin_EXPORTS)
  #define VelodyneHDLPlugin_EXPORT __declspec( dllexport )
 #else
  #define VelodyneHDLPlugin_EXPORT __declspec( dllimport )
 #endif

#else
 #define VelodyneHDLPlugin_EXPORT
#endif


#endif // __vvConfigure_h
