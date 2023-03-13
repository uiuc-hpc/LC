#ifndef _LC_PM_H_
#define _LC_PM_H_

#include "config.h"
#ifdef LCI_USE_PMI2
#include "pmi2.h"
#else
#include "pmi.h"
#endif

void lc_pm_master_init(int* size, int* rank, char* name);
void lc_pm_publish(int prank, int erank, char* value);
void lc_pm_getname(int prank, int erank, char* value);
void lc_pm_barrier();
void lc_pm_finalize();
#endif
