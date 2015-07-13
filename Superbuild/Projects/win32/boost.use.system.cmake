set(BOOST_INCLUDEDIR "${BOOST_INCLUDEDIR}" CACHE PATH "Location of boost header files")
set(BOOST_LIBRARYDIR "${BOOST_LIBRARYDIR}" CACHE PATH "Location of boost library files")

set(Boost_NO_BOOST_CMAKE ON)
find_package(Boost REQUIRED)

# Here we make the assumption that setting these paths will work.  This might fail
# if boost was found via registry entry or BOOST_ROOT entry instead.  We are also
# not checking if required packages in boost were found.
add_extra_cmake_args(
  -DBOOST_INCLUDEDIR:PATH=${BOOST_INCLUDEDIR}
  -DBOOST_LIBRARYDIR:PATH=${BOOST_LIBRARYDIR}
)
