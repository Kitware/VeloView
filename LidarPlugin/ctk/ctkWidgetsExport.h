

// .NAME __ctkCoreExport - manage Windows system differences
// .SECTION Description
// The __ctkCoreExport captures some system differences between Unix
// and Windows operating systems. 

#ifndef __ctkWidgetsExport_h
#define __ctkWidgetsExport_h

#include <QtGlobal>

#if defined(Q_OS_SYMBIAN)
#  if defined(CTKWidgets_EXPORTS)
#    define CTK_WIDGETS_EXPORT Q_DECL_EXPORT
#  else
#    define CTK_WIDGETS_EXPORT Q_DECL_IMPORT
#  endif
#endif

#if !defined(CTK_WIDGETS_EXPORT)
//#  if defined(CTK_SHARED)
#    define CTK_WIDGETS_EXPORT Q_DECL_EXPORT
//#  else
//#    define CTK_WIDGETS_EXPORT
//#  endif
#endif

#endif

