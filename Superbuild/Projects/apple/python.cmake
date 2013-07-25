message(STATUS "Using system python. Pick correct python based on your deployment target")
add_external_project_or_use_system(python)
set(USE_SYSTEM_python TRUE CACHE BOOL "" FORCE)
# FIXME: We automate the picking of the correct python version.
