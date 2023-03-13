configure_file(config.h.in config.h @ONLY)
target_include_directories(lci-obj PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
