cmake_minimum_required(VERSION 3.13)
project(LCI
  VERSION 0.2.0
  DESCRIPTION "Lightweight Communication Interface"
  HOMEPAGE_URL "https://github.com/uiuc-hpc/LC"
  LANGUAGES C
)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(LC_SERVER psm CACHE STRING "Fabric")
set_property(CACHE LC_SERVER PROPERTY STRINGS ofi ibv psm)
option(LC_SERVER_DEBUG "Server debug" OFF)
option(LC_USE_DREG "Use registration cache (only affects ibv -- only use when not-so-dynamic allocation)" ON)
set(LC_EP_AR dynamic explicit CACHE STRING "Addressing mode (dynamic, explicit, immediate)")
set(LC_EP_CE sync cq CACHE STRING "Completion mechanism (sync, cq, am, glob)")
set(LC_MAX_DEV 1 CACHE STRING "Maximum number of devices")
set(LC_MAX_EP 8 CACHE STRING "Maximum number of EP")

if(LC_SERVER STREQUAL "ofi")
  set(LC_USE_SERVER_OFI ON)
elseif(LC_SERVER STREQUAL "ibv")
  set(LC_USE_SERVER_IBV ON)
elseif(LC_SERVER STREQUAL "psm")
  set(LC_USE_SERVER_PSM ON)
else()
  message(FATAL_ERROR "Fabric ${LC_SERVER} not supported")
endif()

if("dynamic" IN_LIST LC_EP_AR)
  set(LC_SERVER_HAS_DYN ON)
endif()
if("explicit" IN_LIST LC_EP_AR)
  set(LC_SERVER_HAS_EXP ON)
endif()
if("immediate" IN_LIST LC_EP_AR)
  set(LC_SERVER_HAS_IMM ON)
endif()

if("sync" IN_LIST LC_EP_CE)
  set(LC_SERVER_HAS_SYNC ON)
endif()
if("cq" IN_LIST LC_EP_CE)
  set(LC_SERVER_HAS_CQ ON)
endif()
if("am" IN_LIST LC_EP_CE)
  set(LC_SERVER_HAS_AM ON)
endif()
if("glob" IN_LIST LC_EP_CE)
  set(LC_SERVER_HAS_GLOB ON)
endif()

set(USE_AFFI ON CACHE BOOL "CPU affinity")
mark_as_advanced(USE_AFFI)
if(USE_AFFI)
  set(AFF_DEBUG ON)
  add_compile_definitions(USE_AFFI AFF_DEBUG)
endif()

set(USE_DREG ${LC_USE_DREG})

#NOTE(danghvu): These numbers are tweaked for performance and some alignment.
#Update at our own risk.
if(LC_USE_SERVER_OFI)
  set(LC_PACKET_SIZE "(32 * 1024 + 4096)")
else()
  set(LC_PACKET_SIZE "(8 * 1024 + 4096)")
endif()
set(LC_PACKET_SIZE ${LC_PACKET_SIZE} CACHE STRING "Size of packet")
set(LC_MAX_INLINE 128 CACHE STRING "Max inline message size")
set(LC_DEV_MEM_SIZE "(8*1024*1024)" CACHE STRING "Size of device memory")
set(LC_SERVER_MAX_RCVS 64 CACHE STRING "Max posted recvs")
set(LC_SERVER_NUM_PKTS 1024 CACHE STRING "Number of packets")
set(LC_CACHE_LINE 64 CACHE STRING "Size of cache line (bytes)")
mark_as_advanced(
  LC_PACKET_SIZE
  LC_MAX_INLINE
  LC_DEV_MEM_SIZE
  LC_SERVER_MAX_RCVS
  LC_SERVER_NUM_PKTS
  LC_CACHE_LINE
)




set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

string(TOUPPER ${LC_SERVER} FABRIC_PREFER)
find_package(Fabric REQUIRED)



add_library(lci-obj OBJECT)
set_target_properties(lci-obj PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  C_VISIBILITY_PRESET       hidden
  C_STANDARD                99
  C_EXTENSIONS              ON
)
target_compile_definitions(lci-obj PRIVATE _GNU_SOURCE)
target_include_directories(lci-obj PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(lci-obj PRIVATE
  Threads::Threads
  Fabric::Fabric
)

add_library(lci_shared SHARED)
add_library(lci_static STATIC)
target_link_libraries(lci_shared PUBLIC lci-obj)
target_link_libraries(lci_static PUBLIC lci-obj)
set_target_properties(lci_shared lci_static PROPERTIES OUTPUT_NAME lci)

add_subdirectory(src)
add_subdirectory(examples)



set(CMAKE_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${CMAKE_PROJECT_NAME}")
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/LCIConfigVersion.cmake"
  COMPATIBILITY ExactVersion
)
configure_package_config_file(LCIConfig.cmake.in LCIConfig.cmake
  INSTALL_DESTINATION "${CMAKE_INSTALL_CMAKEDIR}"
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)
set(PKGCONFIG_REQUIRES_PRIVATE ${Fabric_${FABRIC_PREFER}_PC_Requires})
set(PKGCONFIG_LIBS_PRIVATE ${Fabric_${FABRIC_PREFER}_PC_Libs})
configure_file(liblci.pc.in liblci.pc @ONLY)

install(TARGETS lci_shared lci_static
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h"
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/liblci.pc"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/LCIConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/LCIConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_CMAKEDIR}
)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/FindFabric.cmake"
        DESTINATION ${CMAKE_INSTALL_CMAKEDIR}/cmake_modules
)
install(PROGRAMS lcrun DESTINATION ${CMAKE_INSTALL_BINDIR})
