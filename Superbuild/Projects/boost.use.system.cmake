set(BOOST_INCLUDEDIR "${BOOST_INCLUDEDIR}" CACHE PATH "Location of boost header files" FORCE)
set(BOOST_LIBRARYDIR "${BOOST_LIBRARYDIR}" CACHE PATH "Location of boost library files" FORCE)

set(Boost_NO_BOOST_CMAKE ON)
find_package(Boost REQUIRED)

# This is a hack.  If boost was found by some other mechanism then the
# assumptions here would fail.
add_extra_cmake_args(
  -DBOOST_INCLUDEDIR:PATH=${BOOST_INCLUDEDIR}
  -DBOOST_LIBRARYDIR:PATH=${BOOST_LIBRARYDIR}
)
