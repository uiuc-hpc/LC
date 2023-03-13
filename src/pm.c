#include <stdlib.h>
#include <stdio.h>

#include "lci.h"
#include "pm.h"

extern char lcg_name[];

void lc_pm_master_init(int* size, int* rank, char* name)
{
  int spawned;
#ifdef LCI_USE_PMI2
  int appnum;
  PMI2_Init(&spawned, size, rank, &appnum);
  PMI2_Job_GetId(name, 255);
#else
  PMI_Init(&spawned, size, rank);
  PMI_KVS_Get_my_name(name, 255);
#endif
}

void lc_pm_publish_key(char* key, char* value)
{
#ifdef LCI_USE_PMI2
  PMI2_KVS_Put(key, value);
  PMI2_KVS_Fence();
#else
  PMI_KVS_Put(lcg_name, key, value);
  PMI_Barrier();
#endif
}

void lc_pm_getname_key(char* key, char* value)
{
#ifdef LCI_USE_PMI2
  int vallen;
  PMI2_KVS_Get(lcg_name, PMI2_ID_NULL, key, value, 255, &vallen);
#else
  PMI_KVS_Get(lcg_name, key, value, 255);
#endif
}

void lc_pm_publish(int rank, int gid, char* value)
{
  char key[256];
  sprintf(key, "_LC_KEY_%d_%d", rank, gid);
  lc_pm_publish_key(key, value);
}

void lc_pm_getname(int rank, int gid, char* value)
{
  char key[256];
  sprintf(key, "_LC_KEY_%d_%d", rank, gid);
  lc_pm_getname_key(key, value);
}

void lc_pm_barrier() {
#ifdef LCI_USE_PMI2
  PMI2_KVS_Fence();
#else
  PMI_Barrier();
#endif
}

void lc_pm_finalize() {
#ifdef LCI_USE_PMI2
  PMI2_Finalize();
#else
  PMI_Finalize();
#endif
}