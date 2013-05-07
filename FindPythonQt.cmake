# Find PythonQt
#
# Sets PYTHONQT_FOUND, PYTHONQT_INCLUDE_DIR, PYTHONQT_LIBRARY, PYTHONQT_LIBRARIES
#

find_path(PYTHONQT_INCLUDE_DIR PythonQt.h DOC "Path to the PythonQt include directory")
find_library(PYTHONQT_LIBRARY PythonQt DOC "The PythonQt library")

find_library(PYTHONQTPLUGIN_LIBRARY pqPythonQtPlugin DOC "The pqPythonQtPlugin library")

set(PYTHONQT_FOUND 0)
if (PYTHONQT_INCLUDE_DIR AND PYTHONQT_LIBRARY)
  set(PYTHONQT_FOUND 1)
  set(PYTHONQT_LIBRARIES ${PYTHONQT_LIBRARY})
endif()
