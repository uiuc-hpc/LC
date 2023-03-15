#ifndef LCI_PMI_WRAPPER_H
#define LCI_PMI_WRAPPER_H

#include "runtime/lcii.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define LCM_PMI_STRING_LIMIT 255

void lcm_pm_initialize();
int lcm_pm_initialized();
int lcm_pm_get_rank();
int lcm_pm_get_size();
void lcm_pm_publish(char* key, char* value);
void lcm_pm_getname(int rank, char* key, char* value);
void lcm_pm_barrier();
void lcm_pm_finalize();

struct LCM_PM_ops_t {
  int (*check_availability)();
  void (*initialize)();
  int (*is_initialized)();
  int (*get_rank)();
  int (*get_size)();
  void (*publish)(char* key, char* value);
  void (*getname)(int rank, char* key, char* value);
  void (*barrier)();
  void (*finalize)();
};

void lcm_pm_local_setup_ops(struct LCM_PM_ops_t* ops);

void lcm_pm_pmi1_setup_ops(struct LCM_PM_ops_t* ops);

void lcm_pm_pmi2_setup_ops(struct LCM_PM_ops_t* ops);

#ifdef LCI_PM_BACKEND_ENABLE_PMIX
void lcm_pm_pmix_setup_ops(struct LCM_PM_ops_t* ops);
#endif

#ifdef LCI_PM_BACKEND_ENABLE_MPI
void lcm_pm_mpi_setup_ops(struct LCM_PM_ops_t* ops);
#endif

#if defined(__cplusplus)
}
#endif
#endif
