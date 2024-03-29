#include "lci.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>

#include "comm_exp.h"

int total = TOTAL;

int main(int argc, char** args)
{
  LCI_initialize();
  LCI_endpoint_t ep = LCI_UR_ENDPOINT;  // we can directly use the default ep

  int rank = LCI_RANK;
  int peer_rank = ((1 - LCI_RANK % 2) + LCI_RANK / 2 * 2) % LCI_NUM_PROCESSES;
  LCI_tag_t tag = 99;

  LCI_short_t src;
  *(int*)&src = rank;
  LCI_request_t request;
  LCI_barrier();

  if (rank % 2 == 0) {
    for (int i = 0; i < total; i++) {
      while (LCI_puts(ep, src, peer_rank, tag, LCI_DEFAULT_COMP_REMOTE) ==
             LCI_ERR_RETRY)
        LCI_progress(LCI_UR_DEVICE);
      while (LCI_queue_pop(LCI_UR_CQ, &request) == LCI_ERR_RETRY)
        LCI_progress(LCI_UR_DEVICE);
      assert(*(int*)&request.data.immediate == peer_rank);
    }
  } else {
    for (int i = 0; i < total; i++) {
      while (LCI_queue_pop(LCI_UR_CQ, &request) == LCI_ERR_RETRY)
        LCI_progress(LCI_UR_DEVICE);
      assert(*(int*)&request.data.immediate == peer_rank);
      while (LCI_puts(ep, src, peer_rank, tag, LCI_DEFAULT_COMP_REMOTE) ==
             LCI_ERR_RETRY)
        LCI_progress(LCI_UR_DEVICE);
    }
  }
  LCI_finalize();
  return 0;
}
