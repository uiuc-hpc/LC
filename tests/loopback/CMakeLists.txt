add_lci_tests(
  TESTS
  pt2pts.c
  pt2ptm.c
  pt2ptm_mt.cpp
  pt2ptl.c
  pt2ptl_dyn.c
  pt2ptmn.c
  putla.c
  putva.c
  # puti-self.c putbc-self.c
  LABELS
  loopback
  COMMANDS
  "${SRUN_EXE} -n 1 ${SRUN_EXTRA_ARG} [TARGET]"
  DEPENDENCIES
  OpenMP::OpenMP_CXX)
