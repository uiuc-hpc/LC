target_include_directories(lci-obj PRIVATE include)
target_sources_relative(lci-obj PRIVATE
  1sided.c
  config.c
  cq.c
  lci.c
  lcrq.c
  misc.c
  mt.c
  plist.c
  pm.c
  pool.c
  pt2pt.c
  ${CMAKE_CURRENT_SOURCE_DIR}/$<$<BOOL:${LCI_SYNC_ABT}>:sync_abt.c>
  ${CMAKE_CURRENT_SOURCE_DIR}/$<$<BOOL:${LCI_SYNC_FLAG}>:sync_flag.c>
)

if(LCI_USE_DREG)
  add_subdirectory(dreg)
endif()
add_subdirectory(pmi)
add_subdirectory(include)
