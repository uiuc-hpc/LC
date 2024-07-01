#include "runtime/lcii.h"

static int g_endpoint_num = 0;
int g_next_rdma_key = 0;

static struct fi_info* search_for_prov(struct fi_info* info, char* prov_name)
{
  struct fi_info* cur;

  for (cur = info; cur; cur = cur->next) {
    if (strcmp(cur->fabric_attr->prov_name, prov_name) == 0) {
      return cur;
    }
  }
  return NULL;
}

void LCISD_server_init(LCIS_server_t* s)
{
  LCISI_server_t* server = LCIU_malloc(sizeof(LCISI_server_t));
  *s = (LCIS_server_t)server;

  // Create hint.
  char* p = getenv("LCI_OFI_PROVIDER_HINT");
#ifdef LCI_OFI_PROVIDER_HINT_DEFAULT
  if (p == NULL) {
    p = LCI_OFI_PROVIDER_HINT_DEFAULT;
  }
#endif
  char* prov_name_hint = NULL;
  if (p != NULL) {
    prov_name_hint = malloc(strlen(p) + 1);
    // we don't need to explicitly free prov_name_hint later.
    // fi_freeinfo(hints) will help us free it.
    strcpy(prov_name_hint, p);
  }
  struct fi_info* hints;
  hints = fi_allocinfo();
  hints->fabric_attr->prov_name = prov_name_hint;
  hints->ep_attr->type = FI_EP_RDM;
  //  hints->domain_attr->mr_mode = FI_MR_BASIC;
  hints->domain_attr->mr_mode = FI_MR_VIRT_ADDR | FI_MR_ALLOCATED |
                                FI_MR_PROV_KEY | FI_MR_LOCAL | FI_MR_ENDPOINT;
  hints->domain_attr->threading = FI_THREAD_SAFE;
  hints->domain_attr->control_progress = FI_PROGRESS_MANUAL;
  hints->domain_attr->data_progress = FI_PROGRESS_MANUAL;
  hints->caps = FI_RMA | FI_MSG;
  hints->mode = FI_LOCAL_MR;

  // Create info.
  struct fi_info* all_infos;
  FI_SAFECALL(fi_getinfo(FI_VERSION(1, 6), NULL, NULL, 0, hints, &all_infos));
  // According to the libfabric documentation, fi_getinfo call should
  // return the endpoints that are highest performing first.
  // But it appears cxi provider doesn't follow this rule,
  // so we have to do the search ourselves.
  struct fi_info* cxi_info = search_for_prov(all_infos, "cxi");
  if (cxi_info) {
    // Found the cxi provider.
    server->info = fi_dupinfo(cxi_info);
  } else {
    // Just use the first info.
    server->info = fi_dupinfo(all_infos);
  }
  fi_freeinfo(all_infos);
  LCI_Log(LCI_LOG_INFO, "ofi", "Provider name: %s\n",
          server->info->fabric_attr->prov_name);
  LCI_Log(LCI_LOG_INFO, "ofi", "MR mode hints: [%s]\n",
          fi_tostr(&(hints->domain_attr->mr_mode), FI_TYPE_MR_MODE));
  LCI_Log(LCI_LOG_INFO, "ofi", "MR mode provided: [%s]\n",
          fi_tostr(&(server->info->domain_attr->mr_mode), FI_TYPE_MR_MODE));
  LCI_Log(LCI_LOG_INFO, "ofi", "Thread mode: %s\n",
          fi_tostr(&(server->info->domain_attr->threading), FI_TYPE_THREADING));
  LCI_Log(LCI_LOG_INFO, "ofi", "Control progress mode: %s\n",
          fi_tostr(&(server->info->domain_attr->control_progress),
                   FI_TYPE_PROGRESS));
  LCI_Log(
      LCI_LOG_INFO, "ofi", "Data progress mode: %s\n",
      fi_tostr(&(server->info->domain_attr->data_progress), FI_TYPE_PROGRESS));
  LCI_Log(LCI_LOG_INFO, "ofi", "Capacities: %s\n",
          fi_tostr(&(server->info->caps), FI_TYPE_CAPS));
  LCI_Log(LCI_LOG_INFO, "ofi", "Mode: %s\n",
          fi_tostr(&(server->info->mode), FI_TYPE_MODE));
  LCI_Log(LCI_LOG_INFO, "ofi", "Fi_info provided: %s\n",
          fi_tostr(server->info, FI_TYPE_INFO));
  LCI_Log(LCI_LOG_INFO, "ofi", "Fabric attributes: %s\n",
          fi_tostr(server->info->fabric_attr, FI_TYPE_FABRIC_ATTR));
  LCI_Log(LCI_LOG_INFO, "ofi", "Domain attributes: %s\n",
          fi_tostr(server->info->domain_attr, FI_TYPE_DOMAIN_ATTR));
  LCI_Log(LCI_LOG_INFO, "ofi", "Endpoint attributes: %s\n",
          fi_tostr(server->info->ep_attr, FI_TYPE_EP_ATTR));
  LCI_Assert(server->info->domain_attr->cq_data_size >= 4,
             "cq_data_size (%lu) is too small!\n",
             server->info->domain_attr->cq_data_size);
  LCI_Assert(server->info->domain_attr->mr_key_size <= 8,
             "mr_key_size (%lu) is too large!\n",
             server->info->domain_attr->mr_key_size);
  LCI_Assert(server->info->tx_attr->inject_size >= sizeof(LCI_short_t),
             "inject_size (%lu) < sizeof(LCI_short_t) (%lu)!\n",
             server->info->tx_attr->inject_size, sizeof(LCI_short_t));
  fi_freeinfo(hints);
  if (strcmp(server->info->fabric_attr->prov_name, "cxi") == 0) {
    LCI_Assert(LCI_USE_DREG == 0,
               "The registration cache should be turned off "
               "for libfabric cxi backend. Use `export LCI_USE_DREG=0`.\n");
    LCI_Assert(LCI_ENABLE_PRG_NET_ENDPOINT == 0,
               "The progress-specific network endpoint "
               "for libfabric cxi backend. Use `export "
               "LCI_ENABLE_PRG_NET_ENDPOINT=0`.\n");
    if (LCI_RDV_PROTOCOL != LCI_RDV_WRITE) {
      LCI_RDV_PROTOCOL = LCI_RDV_WRITE;
      LCI_Warn(
          "Switch LCI_RDV_PROTOCOL to \"write\" "
          "as required by the libfabric cxi backend\n");
    }
  }

  // Create libfabric obj.
  FI_SAFECALL(fi_fabric(server->info->fabric_attr, &server->fabric, NULL));
}

void LCISD_server_fina(LCIS_server_t s)
{
  LCISI_server_t* server = (LCISI_server_t*)s;
  FI_SAFECALL(fi_close((struct fid*)&server->fabric->fid));
  fi_freeinfo(server->info);
  free(s);
}

void LCISD_endpoint_init(LCIS_server_t server_pp, LCIS_endpoint_t* endpoint_pp,
                         bool single_threaded)
{
  int endpoint_id = g_endpoint_num++;
  LCISI_endpoint_t* endpoint_p = LCIU_malloc(sizeof(LCISI_endpoint_t));
  *endpoint_pp = (LCIS_endpoint_t)endpoint_p;
  endpoint_p->server = (LCISI_server_t*)server_pp;
  endpoint_p->is_single_threaded = single_threaded;

  // Create domain.
  FI_SAFECALL(fi_domain(endpoint_p->server->fabric, endpoint_p->server->info,
                        &endpoint_p->domain, NULL));

  // Create end-point;
  endpoint_p->server->info->tx_attr->size = LCI_SERVER_MAX_SENDS;
  endpoint_p->server->info->rx_attr->size = LCI_SERVER_MAX_RECVS;
  FI_SAFECALL(fi_endpoint(endpoint_p->domain, endpoint_p->server->info,
                          &endpoint_p->ep, NULL));

  // Create cq.
  struct fi_cq_attr cq_attr;
  memset(&cq_attr, 0, sizeof(struct fi_cq_attr));
  cq_attr.format = FI_CQ_FORMAT_DATA;
  cq_attr.size = LCI_SERVER_MAX_CQES;
  FI_SAFECALL(fi_cq_open(endpoint_p->domain, &cq_attr, &endpoint_p->cq, NULL));

  // Bind my ep to cq.
  FI_SAFECALL(
      fi_ep_bind(endpoint_p->ep, (fid_t)endpoint_p->cq, FI_TRANSMIT | FI_RECV));

  struct fi_av_attr av_attr;
  memset(&av_attr, 0, sizeof(av_attr));
  av_attr.type = FI_AV_MAP;
  FI_SAFECALL(fi_av_open(endpoint_p->domain, &av_attr, &endpoint_p->av, NULL));
  FI_SAFECALL(fi_ep_bind(endpoint_p->ep, (fid_t)endpoint_p->av, 0));
  FI_SAFECALL(fi_enable(endpoint_p->ep));

  // Now exchange end-point address.
  // assume the size of the raw address no larger than 128 bits.
  const int EP_ADDR_LEN = 6;
  size_t addrlen = 0;
  fi_getname((fid_t)endpoint_p->ep, NULL, &addrlen);
  LCI_Log(LCI_LOG_INFO, "ofi", "addrlen = %lu\n", addrlen);
  LCI_Assert(addrlen <= 8 * EP_ADDR_LEN, "addrlen = %lu\n", addrlen);
  uint64_t my_addr[EP_ADDR_LEN];
  FI_SAFECALL(fi_getname((fid_t)endpoint_p->ep, my_addr, &addrlen));

  endpoint_p->peer_addrs = LCIU_malloc(sizeof(fi_addr_t) * LCI_NUM_PROCESSES);
  char key[LCT_PMI_STRING_LIMIT + 1];
  sprintf(key, "LCI_KEY_%d_%d", endpoint_id, LCI_RANK);
  char value[LCT_PMI_STRING_LIMIT + 1];
  const char* PARSE_STRING = "%016lx-%016lx-%016lx-%016lx-%016lx-%016lx";
  sprintf(value, PARSE_STRING, my_addr[0], my_addr[1], my_addr[2], my_addr[3],
          my_addr[4], my_addr[5]);
  LCT_pmi_publish(key, value);
  LCT_pmi_barrier();

  for (int i = 0; i < LCI_NUM_PROCESSES; i++) {
    if (i != LCI_RANK) {
      sprintf(key, "LCI_KEY_%d_%d", endpoint_id, i);
      LCT_pmi_getname(i, key, value);
      uint64_t peer_addr[EP_ADDR_LEN];

      sscanf(value, PARSE_STRING, &peer_addr[0], &peer_addr[1], &peer_addr[2],
             &peer_addr[3], &peer_addr[4], &peer_addr[5]);
      int ret = fi_av_insert(endpoint_p->av, (void*)peer_addr, 1,
                             &endpoint_p->peer_addrs[i], 0, NULL);
      LCI_Assert(ret == 1, "fi_av_insert failed! ret = %d\n", ret);
    } else {
      int ret = fi_av_insert(endpoint_p->av, (void*)my_addr, 1,
                             &endpoint_p->peer_addrs[i], 0, NULL);
      LCI_Assert(ret == 1, "fi_av_insert failed! ret = %d\n", ret);
    }
  }

  LCT_pmi_barrier();
}

void LCISD_endpoint_fina(LCIS_endpoint_t endpoint_pp)
{
  LCT_pmi_barrier();
  LCISI_endpoint_t* endpoint_p = (LCISI_endpoint_t*)endpoint_pp;
  LCIU_free(endpoint_p->peer_addrs);
  FI_SAFECALL(fi_close((struct fid*)&endpoint_p->ep->fid));
  FI_SAFECALL(fi_close((struct fid*)&endpoint_p->cq->fid));
  FI_SAFECALL(fi_close((struct fid*)&endpoint_p->av->fid));
  FI_SAFECALL(fi_close((struct fid*)&endpoint_p->domain->fid));
}