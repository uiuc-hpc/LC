#include <stdlib.h>
#include <stdio.h>

#include "lc.h"
#include "pmi.h"
#include "pm.h"

extern char lcg_name[256];

void lc_pm_master_init(int* size, int* rank, char* name)
{
  int spawned;
  PMI_Init(&spawned, size, rank);
  PMI_KVS_Get_my_name(name, 255);
}

void lc_pm_publish(int rank, int gid, char* value)
{
  char key[256];
  sprintf(key, "_LC_KEY_%d_%d", rank, gid);
  PMI_KVS_Put(lcg_name, key, value);
  PMI_Barrier();
}

void lc_pm_getname(int rank, int gid, char* value)
{
  char key[256];
  sprintf(key, "_LC_KEY_%d_%d", rank, gid);
  PMI_KVS_Get(lcg_name, key, value, 255);
}

void lc_pm_getname_key(char* key, char* value)
{
  PMI_KVS_Get(lcg_name, key, value, 255);
}

void lc_pm_publish_key(char* key, char* value)
{
  PMI_KVS_Put(lcg_name, key, value);
}

void lc_pm_barrier(void) { PMI_Barrier(); }
