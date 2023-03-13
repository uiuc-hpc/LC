#[=======================================================================[.rst:
FindFabric
----------

Finds a low-level fabric library, such as libfabric, libibverbs, or libpsm2.

Components
^^^^^^^^^^

Each library can be specified as a component.
The following components are supported: OFI, IBV, and PSM.
By default, this module will search for all components.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``Fabric::OFI``
  The OpenFabrics Interfaces user-space library

``Fabric::IBV``
  The InfiniBand Verbs library

``Fabric::PSM``
  The Intel Performance Scaled Messaging 2 library

``Fabric::Fabric``
  INTERFACE IMPORTED library that links to ``Fabric::${FABRIC_PREFER}``
  if the caller sets FABRIC_PREFER

Result Variables
^^^^^^^^^^^^^^^^

``FABRIC_FOUND``
  If components have been specified,
  this will be set only if all components have been found;
  otherwise, this is set if any fabric was found.

This module will set the following variables per fabric,
where ``fabric`` is one of OFI, IBV, or PSM:

``FABRIC_<fabric>_FOUND``
  True if ``<fabric>`` was found

``FABRIC_<fabric>_INCLUDE_DIRS``
  Include directories needed to use ``<fabric>``

``FABRIC_<fabric>_LIBRARIES``
  Libraries needed to link to ``<fabric>``

``Fabric_<fabric>_CFLAGS_OTHER``
  Other CFLAGS needed to use ``<fabric>``

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``FABRIC_<fabric>_INCLUDE_DIR``
  The include directory for ``<fabric>``

``FABRIC_<fabric>_LIBRARY``
  Path to the library for ``<fabric>``

#]=======================================================================]

include(FindPackageHandleStandardArgs)
find_package(PkgConfig)

set(_OFI_PC libfabric)
set(_OFI_HEADER "rdma/fabric.h")
set(_OFI_LIB fabric)

set(_IBV_PC libibverbs)
set(_IBV_HEADER "infiniband/verbs.h")
set(_IBV_LIB ibverbs)

set(_PSM_PC libpsm2)
set(_PSM_HEADER "psm2.h")
set(_PSM_LIB psm2)

foreach(FABRIC IN ITEMS OFI IBV PSM)
  if(NOT Fabric_FIND_COMPONENTS OR ${FABRIC} IN_LIST Fabric_FIND_COMPONENTS)
    pkg_check_modules(_Fabric_${FABRIC}_PC QUIET ${_${FABRIC}_PC})
    find_path(Fabric_${FABRIC}_INCLUDE_DIR
              NAMES ${_${FABRIC}_HEADER}
              PATHS ${_Fabric_${FABRIC}_PC_INCLUDE_DIRS}
    )
    find_library(Fabric_${FABRIC}_LIBRARY
                 NAMES ${_${FABRIC}_LIB}
                 PATHS ${_Fabric_${FABRIC}_PC_LIBRARY_DIRS}
    )
    set(Fabric_${FABRIC}_VERSION ${_Fabric_${FABRIC}_PC_VERSION})

    find_package_handle_standard_args(Fabric_${FABRIC}
      REQUIRED_VARS
        Fabric_${FABRIC}_INCLUDE_DIR
        Fabric_${FABRIC}_LIBRARY
      VERSION_VAR Fabric_${FABRIC}_VERSION
    )

    if(Fabric_${FABRIC}_FOUND)
      set(Fabric_${FABRIC}_INCLUDE_DIRS ${Fabric_${FABRIC}_INCLUDE_DIR})
      set(Fabric_${FABRIC}_LIBRARIES ${Fabric_${FABRIC}_LIBRARY})
      set(Fabric_${FABRIC}_CFLAGS_OTHER ${_Fabric_${FABRIC}_PC_CFLAGS_OTHER})
      if(_Fabric_${FABRIC}_PC_FOUND)
        set(Fabric_${FABRIC}_PC_Requires ${_${FABRIC}_PC})
      else()
        set(Fabric_${FABRIC}_PC_Libs "-l${_${FABRIC}_LIB}")
      endif()

      if(NOT TARGET Fabric::${FABRIC})
        add_library(Fabric::${FABRIC} UNKNOWN IMPORTED)
        set_target_properties(Fabric::${FABRIC} PROPERTIES
          IMPORTED_LOCATION "${Fabric_${FABRIC}_LIBRARY}"
          INTERFACE_INCLUDE_DIRECTORIES "${Fabric_${FABRIC}_INCLUDE_DIR}"
          INTERFACE_COMPILE_OPTIONS "${_Fabric_${FABRIC}_PC_CFLAGS_OTHER}"
        )
      endif()

      if(FABRIC STREQUAL FABRIC_PREFER AND NOT TARGET Fabric::Fabric)
        add_library(Fabric::Fabric INTERFACE IMPORTED)
        target_link_libraries(Fabric::Fabric INTERFACE Fabric::${FABRIC})
      endif()
    endif()

    mark_as_advanced(Fabric_${FABRIC}_INCLUDE_DIR Fabric_${FABRIC}_LIBRARY)
  endif()
endforeach()

unset(_FABRIC_REQUIRED_VARS)
foreach(FABRIC IN ITEMS OFI IBV PSM)
  if((NOT Fabric_FIND_COMPONENTS AND FABRIC_${FABRIC}_FOUND) OR
     (FABRIC STREQUAL FABRIC_PREFER) OR
     (FABRIC IN_LIST Fabric_FIND_COMPONENTS AND Fabric_FIND_REQUIRED_${FABRIC}))
    list(APPEND _FABRIC_REQUIRED_VARS "FABRIC_${FABRIC}_FOUND")
  endif()
endforeach()

find_package_handle_standard_args(Fabric
  REQUIRED_VARS ${_FABRIC_REQUIRED_VARS}
  HANDLE_COMPONENTS
)
