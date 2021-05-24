#include "lci.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>

#include "comm_exp.h"

#undef MAX_MSG
#define MAX_MSG 8

int total = TOTAL;

int main(int argc, char** args) {
  LCI_open();
  LCI_endpoint_t ep = LCI_UR_ENDPOINT; // we can directly use the default ep

  int rank = LCI_RANK;
  int peer_rank = ((1 - LCI_RANK % 2) + LCI_RANK / 2 * 2) % LCI_NUM_PROCESSES;
  LCI_tag_t tag = 99;

  LCI_short_t src = rank;
  LCI_request_t request;
  LCI_barrier();

  if (rank % 2 == 0) {
    for (int i = 0; i < total; i++) {
      LCI_puts(ep, src, peer_rank, tag, LCI_UR_CQ_REMOTE);
      while (LCI_queue_pop(LCI_UR_CQ, &request) == LCI_ERR_RETRY)
        LCI_progress(0, 1);
      assert(request.data.immediate == peer_rank);
    }
  } else {
    for (int i = 0; i < total; i++) {
      while (LCI_queue_pop(LCI_UR_CQ, &request) == LCI_ERR_RETRY)
        LCI_progress(0, 1);
      assert(request.data.immediate == peer_rank);
      LCI_puts(ep, src, peer_rank, tag, LCI_UR_CQ_REMOTE);
    }
  }
  LCI_close();
  return 0;
}