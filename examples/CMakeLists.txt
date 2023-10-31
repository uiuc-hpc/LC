function(add_example NAME)
  add_executable(${NAME} ${NAME}.c)
  target_compile_definitions(${NAME} PRIVATE _GNU_SOURCE)
  target_link_libraries(${NAME} PRIVATE
    Threads::Threads
    lci_static
  )
  target_link_options(${NAME} PRIVATE
    LINKER:-z,now
    LINKER:-z,relro
  )
  set_target_properties(${NAME} PROPERTIES
    C_STANDARD                99
    C_EXTENSIONS              ON
  )
  install(TARGETS ${NAME} DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/lc)
endfunction()

function(add_all_examples)
  foreach(example ${ARGN})
    add_example(${example})
  endforeach()
endfunction()

add_all_examples(
  init
  tshort
  tmed
  tlong
  qshort
  qmed
  qlong
  qmed_dup
  qmed_dup_2hw
  puts
  putm
  putl
  putss
  putms
  putls
  putsam
  tag
  ibarrier
  ialreduce
)
