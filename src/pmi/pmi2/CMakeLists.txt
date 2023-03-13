target_include_directories(lci-obj PRIVATE include)
target_sources_relative(lci-obj PRIVATE
        client/pmi2_util.c
        client/pmi2_api.c
        client/pmi2_util.h
        )

add_executable(pmi2-test client/test.c
        client/pmi2_util.c
        client/pmi2_api.c
        client/pmi2_util.h
        )
target_include_directories(pmi2-test PRIVATE include)
target_compile_definitions(pmi2-test PRIVATE _GNU_SOURCE)
set_target_properties(pmi2-test PROPERTIES C_STANDARD 99 C_EXTENSIONS ON)
