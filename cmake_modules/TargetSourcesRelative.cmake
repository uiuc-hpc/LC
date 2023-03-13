# Acknowledgement: modified from https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/
# NOTE: This helper function assumes all the generator expressions used
#       for the source files are written in absolute paths
function(target_sources_relative target)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.13")
    cmake_policy(PUSH)
    cmake_policy(SET CMP0076 NEW)
    target_sources(${target} ${ARGN})
    cmake_policy(POP)
  else()
    unset(_srcList)
    get_target_property(_targetSourceDir ${target} SOURCE_DIR)

    foreach(src ${ARGN})
      if(NOT src STREQUAL "PRIVATE" AND
              NOT src STREQUAL "PUBLIC" AND
              NOT src STREQUAL "INTERFACE" AND
              NOT IS_ABSOLUTE "${src}")
        # Relative path to source, prepend relative to where target was defined
        file(RELATIVE_PATH src "${_targetSourceDir}" "${CMAKE_CURRENT_LIST_DIR}/${src}")
      endif()
      list(APPEND _srcList ${src})
    endforeach()
    target_sources(${target} ${_srcList})
  endif()
endfunction()