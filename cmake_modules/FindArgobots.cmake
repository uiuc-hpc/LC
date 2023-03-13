#[=======================================================================[.rst:
FindArgobots
----------

Finds Argobots

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``Argobots::Argobots``
  The Argobots library

Result Variables
^^^^^^^^^^^^^^^^

``Argobots_FOUND``
  Set if Argobots was found

``Argobots_INCLUDE_DIRS``
  Include directories needed to use Argobots

``Argobots_LIBRARIES``
  Libraries needed to link to Argobots

``Argobots_CFLAGS_OTHER``
  Other CFLAGS needed to use Argobots

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``Argobots_INCLUDE_DIR``
  The include directory for Argobots

``Argobots_LIBRARY``
  Path to the library for Argobots

#]=======================================================================]

include(FindPackageHandleStandardArgs)
find_package(PkgConfig)

pkg_check_modules(_Argobots_PC QUIET argobots)
find_path(Argobots_INCLUDE_DIR
          NAMES "abt.h"
          PATHS ${_Argobots_PC_INCLUDE_DIRS}
)
find_library(Argobots_LIBRARY
             NAMES "abt"
             PATHS ${_Argobots_PC_LIBRARY_DIRS}
)
set(Argobots_VERSION ${_Argobots_PC_VERSION})

find_package_handle_standard_args(Argobots
  REQUIRED_VARS
    Argobots_INCLUDE_DIR
    Argobots_LIBRARY
  VERSION_VAR Argobots_VERSION
)

if(Argobots_FOUND)
  set(Argobots_INCLUDE_DIRS ${Argobots_INCLUDE_DIR})
  set(Argobots_LIBRARIES ${Argobots_LIBRARY})
  set(Argobots_CFLAGS_OTHER ${_Argobots_PC_CFLAGS_OTHER})

  if(NOT TARGET Argobots::Argobots)
    add_library(Argobots::Argobots UNKNOWN IMPORTED)
    set_target_properties(Argobots::Argobots PROPERTIES
      IMPORTED_LOCATION "${Argobots_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "{Argobots_INCLUDE_DIR}"
      INTERFACE_COMPILE_OPTIONS "${_Argobots_PC_CFLAGS_OTHER}"
    )
  endif()

  mark_as_advanced(Argobots_INCLUDE_DIR Argobots_LIBRARY)
endif()
