target_include_directories(lci-obj PRIVATE include)
target_sources(lci-obj PRIVATE
  coll.c
  cq.c
  ep.c
  glob.c
  hashtable.c
  lc.c
  lcrq.c
  long.c
  medium.c
  misc.c
  pool.c
  pm.c
  short.c
  tag.c
)

if(LC_USE_DREG)
  add_subdirectory(dreg)
endif()
add_subdirectory(pmi)
add_subdirectory(include)
