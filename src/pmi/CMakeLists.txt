target_sources(lci-obj PRIVATE
  simple_pmi.c
  simple_pmiutil.c
)

add_executable(pmi-test simple_pmi.c simple_pmiutil.c test.c)
target_include_directories(pmi-test PRIVATE ../include)
target_compile_definitions(pmi-test PRIVATE _GNU_SOURCE)
set_target_properties(pmi-test PROPERTIES C_STANDARD 99 C_EXTENSIONS ON)
