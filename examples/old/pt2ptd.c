#include "lci.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>

#include "../comm_exp.h"

#undef MAX_MSG
#define MAX_MSG (4 * 1024 * 1024)

int total = TOTAL;
int skip = SKIP;

int main(int argc, char** args)
{
  LCI_open();
  LCI_endpoint_t ep;
  LCI_plist_t prop;
  LCI_plist_create(&prop);
  LCI_MT_t mt;
  LCI_MT_init(&mt, 0);
  LCI_plist_set_MT(prop, &mt);
  LCI_endpoint_init(&ep, 0, prop);

  int rank = LCI_RANK;
  LCI_tag_t tag = 99;

  LCI_comp_t sync;

  double t1 = 0;
  size_t alignment = sysconf(_SC_PAGESIZE);
  void* src_buf = 0;
  void* dst_buf = 0;
  posix_memalign(&src_buf, alignment, MAX_MSG);
  posix_memalign(&dst_buf, alignment, MAX_MSG);

  if (rank == 0) {
    for (int size = MIN_MSG; size <= MAX_MSG; size <<= 1) {
      memset(src_buf, 'a', size);
      memset(dst_buf, 'b', size);

      if (size > LARGE) {
        total = TOTAL_LARGE;
        skip = SKIP_LARGE;
      }

      for (int i = 0; i < total + skip; i++) {
        if (i == skip) t1 = wtime();
        LCI_one2one_set_empty(&sync);
        while (LCI_sendl(ep, src_buf, 1 - rank, tag, sync, 0) != LCI_OK)
          LCI_progress(LCI_UR_DEVICE);
        while (LCI_one2one_test_empty(&sync)) LCI_progress(LCI_UR_DEVICE);
        LCI_one2one_set_empty(&sync);
        LCI_recvd(dst_buf, size, 1 - rank, tag, ep, &sync);
        while (LCI_one2one_test_empty(&sync)) LCI_progress(LCI_UR_DEVICE);
        if (i == 0) {
          for (int j = 0; j < size; j++)
            assert(((char*)src_buf)[j] == 'a' && ((char*)dst_buf)[j] == 'a');
        }
      }

      t1 = 1e6 * (wtime() - t1) / total / 2;
      printf("%10.d %10.3f\n", size, t1);
      fflush(stdout);
    }
  } else {
    for (int size = MIN_MSG; size <= MAX_MSG; size <<= 1) {
      memset(src_buf, 'a', size);
      memset(dst_buf, 'b', size);
      if (size > LARGE) {
        total = TOTAL_LARGE;
        skip = SKIP_LARGE;
      }

      for (int i = 0; i < total + skip; i++) {
        LCI_one2one_set_empty(&sync);
        LCI_recvd(dst_buf, size, 1 - rank, tag, ep, &sync);
        while (LCI_one2one_test_empty(&sync)) LCI_progress(LCI_UR_DEVICE);
        LCI_one2one_set_empty(&sync);
        while (LCI_sendl(ep, src_buf, 1 - rank, tag, sync, 0) != LCI_OK)
          LCI_progress(LCI_UR_DEVICE);
        while (LCI_one2one_test_empty(&sync)) LCI_progress(LCI_UR_DEVICE);
      }
    }
  }
  LCI_close();
}
