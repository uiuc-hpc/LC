/**
 * @file lci.h
 * @author Hoang-Vu Dang (danghvu@gmail.com)
 *         Omri Mor (omrimor2@illinois.edu)
 *         Jiakun Yan (jiakuny3@illinois.edu)
 * @brief Header file for all LCI code.
 */

#ifndef LCI_H_
#define LCI_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include "lct.h"
#include "lci_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup LCI_API Lightweight Communication Interface (LCI) API
 * @brief This section describes LCI API.
 */

/**
 * @defgroup LCI_SETUP LCI environment setup
 * @ingroup LCI_API
 * @brief Setup the communication resources and configurations.
 *
 * In order to use LCI, users need to first setup a few communication resources
 * and configurations, including devices and endpoints.
 */

/**
 * @defgroup LCI_DEVICE LCI device
 * @ingroup LCI_SETUP
 * @brief A device is a physical or logical resource that can be used for
 * communication. Communications that use the same device share a resource and
 * may affect each other's performance.
 */

/**
 * @defgroup LCI_PLIST LCI property list
 * @ingroup LCI_SETUP
 * @brief A property list is a collection of properties used to create an
 * endpoint.
 */

/**
 * @defgroup LCI_ENDPOINT LCI endpoint
 * @ingroup LCI_SETUP
 * @brief Currently, a LCI endpoint is not associated with any low-level
 * communication resources, it is just a way to specify a bunch of
 * configurations.
 */

/**
 * @defgroup LCI_COMM LCI communication operations
 * @ingroup LCI_API
 * @brief Communication operations to send and receive messages across
 * processes.
 *
 * The LCI runtime offers users various communication operations for
 * sending and receiving messages. According to the size and number, the
 * messages can be divided into short, medium, long, and iovec messages. LCI
 * provided different operations (with s, m, l, v as suffix) for those messages.
 * LCI uses different protocols for these messages. According to who provides
 * the send and receive buffers, the operations can be divided into 2sided and
 * 1sided operations (send/recv v.s. put/get). For medium send/recv buffers and
 * long recv buffers, the communication operations can either use user-allocated
 * buffer or LCI-provided buffer. The later one can save one memory copy.
 */

/**
 * @defgroup LCI_COMP LCI completion mechanisms
 * @ingroup LCI_API
 * @brief The LCI runtime notifies users about the completion of an asynchronous
 * operations (e.g. communication operations) through completion
 * mechanisms.
 *
 * LCI provides three completion mechanisms: synchronizers,
 * completion queues, and active message handlers.
 */

/**
 * @defgroup LCI_SYNC LCI synchronizer
 * @ingroup LCI_COMP
 * @brief One of the completion mechanisms.
 *
 * In its simplest form, it is an object similar to MPI_Request.
 * Every synchronizer can be associated with one or more
 * asynchronous operations. The synchronizer will be triggered after all the
 * associated operations are completed. Users can know whether the corresponding
 * operations have been completed by querying the synchronizer.
 *
 * It can also be scheduler-aware when executing in the context of
 * a task-based runtime (This is still an ongoing work).
 */

/**
 * @defgroup LCI_QUEUE LCI completion queue
 * @ingroup LCI_COMP
 * @brief One of the completion mechanisms.
 *
 * Every completion queue can be associated with one or more
 * asynchronous operations. When a operation is completed,
 * the LCI runtime will push an entry into the queue, users can
 * get completion notification by calling @ref LCI_cq_pop.
 */

/**
 * @defgroup LCI_HANDLER LCI active message handler
 * @ingroup LCI_COMP
 * @brief One of the completion mechanisms.
 *
 * Every active message handler can be associated with one or more
 * asynchronous operations. When operations are completed,
 * the LCI runtime will execute the handler with the completion entry
 * as the input.
 */

/**
 * @defgroup LCI_PROGRESS LCI progress functions
 * @ingroup LCI_API
 * @brief Make progress on the background works to make sure the runtime
 * functioning properly.
 *
 * All communication runtimes have some background work to do, such as
 * pre-posting some receive buffers to the network, polling the network
 * completion queue, serving for some communication protocols, and executing
 * active message handlers. MPI chooses to do them as a side effect of some MPI
 * function calls (such as MPI_Test, MPI_Wait). LCI chooses to provide users
 * with an explicit function so that users can control when and how to make
 * progress on these background works.
 *
 * Caveat: The progress functions have to be called frequently to make sure
 * all communication operations can complete.
 */

/**
 * @defgroup LCI_MEM LCI memory management
 * @ingroup LCI_API
 * @brief Management for memory buffers registered to the network.
 *
 * In most of the cases, memory buffers participated in a network operations
 * need to be registered to the network. The LCI runtime can do this for
 * users through either memory copies to a pre-registered buffers or dynamical
 * registering and deregistering. LCI also exposes relevant functions regarding
 * registered memory allocation and memory registration to the users so users
 * can do communication operations more efficiently.
 *
 * For small messages, the send data does not need to be registered since the
 * data will be packed into the low-level network operation descriptors.
 *
 * For medium messages, the send and receive buffers need to be registered. If
 * users are using the "copy" version of operations (e.g. @ref LCI_sendm, @ref
 * LCI_putm, @ref LCI_recvm), the data will be copied to/from pre-registered
 * buffers by the runtime. If users are using the "no-copy"version of operations
 * (e.g. @ref LCI_sendmn, @ref LCI_putmn, @ref LCI_recvmn), users will directly
 * get access to the LCI internal pre-registered buffers through either
 * @ref LCI_mbuffer_alloc (for send buffers) or completion mechanisms (for
 * receive buffers).
 *
 * For long messages, the send and receive buffers need to be registered, and
 * the cost of memory copies to pre-registered buffers is considered unworthy.
 * As a result, either users or the runtime needs to explicitly register the
 * buffers.
 */

/**
 * @defgroup LCIX_API LCI Experimental API
 * @brief This section describes LCI experimental APIs that may be under
 * development, poorly documented, or otherwise incomplete.
 */

/**
 * @defgroup LCI_COLL LCI Collectives
 * @ingroup LCIX_API
 * @brief MPI-style collective communications; these are provided for the
 * purpose of compatability and ease of porting to LCI and are not expected to
 * be used in performance-critical code. Collective completion uses the message
 * port. Use this API with caution.
 */

#define LCI_API __attribute__((visibility("default")))

#define LCI_DEFAULT_COMP_REMOTE 0
// "pseudo-segment" indicating the entire address space,
// leading to dynamic (on-the-fly) registration
#define LCI_SEGMENT_ALL NULL
#define LCI_RANK_ANY (-1)

/**
 * @ingroup LCI_API
 * @brief LCI Error type.
 */
typedef enum {
  LCI_OK = 0,         /**< Okay. No error. */
  LCI_ERR_RETRY,      /**< Resource temporarily not available. Try again. */
  LCI_ERR_RETRY_LOCK, /**< Internal use only. Resource temporarily not available
                         due to lock  contention. Try again. */
  LCI_ERR_RETRY_NOMEM, /**< Internal use only. Resource temporarily not
                          available due to no memory. Try again. */
  LCI_ERR_FEATURE_NA,  /**< Feature not available */
  LCI_ERR_FATAL,       /**< Fatal error */
} LCI_error_t;

/**
 * @ingroup LCI_COMM
 * @brief LCI Match type. Define the matching rule between sends and receives.
 */
typedef enum {
  LCI_MATCH_RANKTAG = 0, /**< Match send and receive by
                            (source rank, target rank, tag) */
  LCI_MATCH_TAG,         /**< Match send and receive by (tag) */
} LCI_match_t;

/**
 * @ingroup LCI_COMP
 * @brief LCI Port type.
 */
typedef enum {
  LCI_PORT_COMMAND = 0, /**< The send side */
  LCI_PORT_MESSAGE = 1, /**< The receive side */
} LCI_port_t;

/**
 * @ingroup LCI_COMP
 * @brief LCI completion enumeration type.
 */
typedef enum {
  LCI_COMPLETION_NONE = 0,
  LCI_COMPLETION_QUEUE,    // completion queue
  LCI_COMPLETION_HANDLER,  // handler
  LCI_COMPLETION_SYNC,     // synchronizer
} LCI_comp_type_t;

/**
 * @ingroup LCI_COMP
 * @brief LCI data type.
 */
typedef enum {
  LCI_IMMEDIATE = 0, /**< Immediate data (up to LCI_SHORT_SIZE bytes) sent by
                        short messages */
  LCI_MEDIUM,        /**< Medium buffers (up to LCI_MEDIUM_SIZE bytes) sent by
                        medium messages */
  LCI_LONG,          /**< Long buffers sent by long messages */
  LCI_IOVEC          /**< Iovecs (A medium buffer + multiple long buffers) sent
                        by iovec messages */
} LCI_data_type_t;

/**
 * @ingroup LCI_COMM
 * @brief Tag type.
 *
 * The largest tag allowed by the runtime is defined by LCI_MAX_TAG.
 */
typedef uint32_t LCI_tag_t;

/**
 * @ingroup LCI_COMP
 * @brief LCI generic completion type.
 */
typedef void* LCI_comp_t;

struct LCII_mr_t;
/**
 * @ingroup LCI_MEM
 * @brief LCI memory segment.
 *
 * All LCI communication must take place in memory segments, which represent
 * memory regions registered to devices.
 */
typedef struct LCII_mr_t* LCI_segment_t;

/**
 * @ingroup LCI_COMM
 * @brief LCI long communication buffer.
 *
 * All long buffer passed to a communication operation needs to be registered
 * to the device (or set the segment to LCI_SEGMENT_ALL and LCI will register
 * and deregister the buffer).
 */
struct LCI_lbuffer_t {
  LCI_segment_t segment;
  void* address;
  size_t length;
};
typedef struct LCI_lbuffer_t LCI_lbuffer_t;

/**
 * @ingroup LCI_COMM
 * @brief LCI medium communication buffer.
 *
 * Medium communication buffers reside in memory managed by LCI.
 */
struct LCI_mbuffer_t {
  void* address;
  size_t length;
};
typedef struct LCI_mbuffer_t LCI_mbuffer_t;

/**
 * @ingroup LCI_COMM
 * @brief The size of LCI short data
 */
#define LCI_SHORT_SIZE 32

/**
 * @ingroup LCI_COMM
 * @brief LCI short data.
 */
typedef union {
#ifdef LCI_USE_AVX
  uint8_t u8 __attribute__((vector_size(32)));
  uint16_t u16 __attribute__((vector_size(32)));
  uint32_t u32 __attribute__((vector_size(32)));
  uint64_t u64 __attribute__((vector_size(32)));
  int8_t s8 __attribute__((vector_size(32)));
  int16_t s16 __attribute__((vector_size(32)));
  int32_t s32 __attribute__((vector_size(32)));
  int64_t s64 __attribute__((vector_size(32)));
  double dbl __attribute__((vector_size(32)));
  float flt __attribute__((vector_size(32)));
#else
  char val[LCI_SHORT_SIZE];
#endif
} LCI_short_t;
// typedef uint64_t LCI_short_t;

/**
 * @ingroup LCI_COMM
 * @brief LCI iovec, which is comprised of a piggyback buffer and multiple long
 * buffers.
 *
 * The maximum number of long buffers is defined by @ref LCI_IOVEC_SIZE.
 *
 * The maximum size of the piggyback buffer depends on the number of long
 * buffers and can be queried via @ref LCI_get_iovec_piggy_back_size.
 */
struct LCI_iovec_t {
  LCI_mbuffer_t piggy_back;  // 16 bytes
  LCI_lbuffer_t* lbuffers;   // 8 bytes
  int count;                 // 4 bytes
};
typedef struct LCI_iovec_t LCI_iovec_t;  // 28 bytes

/**
 * @ingroup LCI_COMP
 * @brief A generic type for communication buffers.
 */
typedef union {
  LCI_short_t immediate;  // 32 bytes
  LCI_mbuffer_t mbuffer;  // 16 bytes
  LCI_lbuffer_t lbuffer;  // 24 bytes
  LCI_iovec_t iovec;      // 28 bytes
} LCI_data_t;

/**
 * @ingroup LCI_COMP
 * @brief Request object. Completion mechanisms will write completion
 * information to it.
 */
typedef struct {
  /* Status of the communication. */
  LCI_error_t flag;      // 4 bytes
  int rank;              // 4 bytes
  LCI_tag_t tag;         // 4 bytes
  LCI_data_type_t type;  // 4 bytes
  LCI_data_t data;       // 32 bytes
  void* user_context;    // 8 bytes
} LCI_request_t;

struct LCI_device_s;
/**
 * @ingroup LCI_DEVICE
 * @brief The device type.
 */
typedef struct LCI_device_s* LCI_device_t;

struct LCI_endpoint_s;
/**
 * @ingroup LCI_ENDPOINT
 * @brief The endpoint type.
 */
typedef struct LCI_endpoint_s* LCI_endpoint_t;

struct LCI_plist_s;
/**
 * @ingroup LCI_PLIST
 * @brief The property list type.
 */
typedef struct LCI_plist_s* LCI_plist_t;

/**
 * @ingroup LCI_HANDLER
 * @brief The active message handler type
 */
typedef void (*LCI_handler_t)(LCI_request_t request);

/**
 * @ingroup LCI_SETUP
 * @brief The number of processes in this job.
 */
extern int LCI_NUM_PROCESSES;

/**
 * @ingroup LCI_SETUP
 * @brief The rank of the current process w.r.t the job.
 */
extern int LCI_RANK;

/**
 * @ingroup LCI_ENDPOINT
 * @brief The maximum number of endpoints that can be created.
 */
extern int LCI_MAX_ENDPOINTS;

/**
 * @ingroup LCI_COMM
 * @brief The largest allowed tag value.
 */
extern int LCI_MAX_TAG;

/**
 * @ingroup LCI_COMM
 * @brief The maximum size (in byte) of a buffer that can be used in medium
 * messages.
 */
extern int LCI_MEDIUM_SIZE;

/**
 * @ingroup LCI_COMM
 * @brief The maximum number of long buffers in an IO vector.
 */
extern int LCI_IOVEC_SIZE;

/**
 * @ingroup LCI_SETUP
 * @brief Initial number of entries in a default matching table.
 * @note The matching table width is fixed in current implementation, but the
 * number of entries is unlimited due to list-based buckets.
 */
extern int LCI_DEFAULT_TABLE_LENGTH;

/**
 * @ingroup LCI_SETUP
 * @brief Maximum number of entries in a matching table.
 * @note The matching table width is fixed in current implementation, but the
 * number of entries is unlimited due to list-based buckets.
 */
extern int LCI_MAX_TABLE_LENGTH;

/**
 * @ingroup LCI_QUEUE
 * @brief Initial number of entries in a default completion queue.
 * @note The completion queue size is fixed in current implementation.
 */
extern int LCI_DEFAULT_QUEUE_LENGTH;

/**
 * @ingroup LCI_QUEUE
 * @brief Maximum number of entries in a completion queue.
 * @note The completion queue size is fixed in current implementation.
 */
extern int LCI_MAX_QUEUE_LENGTH;

/**
 * @ingroup LCI_SYNC
 * @brief Maximum number of request a synchronizer may be waiting for.
 * @note Unlimited for now.
 */
extern int LCI_MAX_SYNC_LENGTH;

/**
 * @ingroup LCI_MEM
 * @brief The packet returning threshold.
 *
 * Apply to @ref LCI_sendm and @ref LCI_sendmn.
 * if size > LCI_PACKET_RETURN_THRESHOLD:
 *   the packet will be returned to the compute thread's pool.
 * else:
 *   the packet will be returned to the progress thread's pool.
 */
extern int LCI_PACKET_RETURN_THRESHOLD;

/**
 * @ingroup LCI_MEM
 * @brief Whether or what to use IBV on-demand paging.
 *
 * 0: Don't use ODP
 * 1: Use explicit ODP
 * 2: Use implicit ODP
 */
extern int LCI_IBV_USE_ODP;

/**
 * @ingroup LCI_MEM
 * @brief Whether touch every page of lbuffers allocated by LCI_lbuffer_alloc
 */
extern int LCI_TOUCH_LBUFFER;

/**
 * @ingroup LCI_MEM
 * @brief Whether or what to use LCI-provided registration cache.
 *
 * 0: Don't use.
 * 1: Use.
 */
extern int LCI_USE_DREG;

/**
 * @ingroup LCI_MEM
 * @brief Whether or what to use IBV prefetch.
 *
 * 0: Don't use.
 * 1: Use. (Only make sense when LCI_IBV_USE_ODP is 1 or 2)
 */
extern int LCI_IBV_USE_PREFETCH;

/**
 * @ingroup LCI_MEM
 * @brief Size of each packet.
 */
extern int LCI_PACKET_SIZE;

/**
 * @ingroup LCI_MEM
 * @brief Total number of packets.
 */
extern int LCI_SERVER_NUM_PKTS;

/**
 * @ingroup LCI_DEVICE
 * @brief Server's send queue length
 */
extern int LCI_SERVER_MAX_SENDS;

/**
 * @ingroup LCI_DEVICE
 * @brief Server's recv queue length
 */
extern int LCI_SERVER_MAX_RECVS;

/**
 * @ingroup LCI_DEVICE
 * @brief Server's completion queue length
 */
extern int LCI_SERVER_MAX_CQES;

/**
 * @ingroup LCI_DEVICE
 * @brief whether to enable ibv's event polling thread.
 */
extern bool LCI_IBV_ENABLE_EVENT_POLLING_THREAD;

/**
 * @ingroup LCI_COMM
 * @brief Manually slow down LCI backend send function
 */
extern int LCI_SEND_SLOW_DOWN_USEC;

/**
 * @ingroup LCI_COMM
 * @brief Manually slow down LCI backend recv function
 */
extern int LCI_RECV_SLOW_DOWN_USEC;

/**
 * @ingroup LCI_COMM
 * @brief Whether to enable the per-cq lock (and thread domain if possible)
 * when using IBV backend.
 */
extern bool LCI_IBV_ENABLE_TD;

/**
 * @ingroup LCI_COMM
 * @brief Whether to enable the progress specific network endpoint.
 */
extern bool LCI_ENABLE_PRG_NET_ENDPOINT;

/**
 * @ingroup LCI_COMM
 * @brief Rendezvous protocol to use.
 */
typedef enum {
  LCI_RDV_WRITE,
  LCI_RDV_WRITEIMM,
} LCI_rdv_protocol_t;
extern LCI_rdv_protocol_t LCI_RDV_PROTOCOL;

/**
 * @ingroup LCI_COMM
 * @brief For the UCX backend, use try_lock to wrap the ucx function calls.
 */
extern bool LCI_UCX_USE_TRY_LOCK;

/**
 * @ingroup LCI_COMM
 * @brief For the UCX backend, use blocking lock to wrap the ucx_progress
 * function calls.
 */
extern bool LCI_UCX_PROGRESS_FOCUSED;

/**
 * @ingroup LCI_COMM
 * @brief Try_lock mode of network backend.
 */
typedef enum {
  LCI_BACKEND_TRY_LOCK_SEND = 1,
  LCI_BACKEND_TRY_LOCK_RECV = 1 << 1,
  LCI_BACKEND_TRY_LOCK_POLL = 1 << 2,
  LCI_BACKEND_TRY_LOCK_GLOBAL = 1 << 3,
  LCI_BACKEND_LOCK_GLOBAL = 1 << 4,
  LCI_BACKEND_TRY_LOCK_MODE_MAX = 1 << 5,
} LCI_backend_try_lock_mode_t;
extern uint64_t LCI_BACKEND_TRY_LOCK_MODE;

/**
 * @ingroup LCI_DEVICE
 * @brief Default device initialized by LCI_initialize. Just for convenience.
 */
extern LCI_device_t LCI_UR_DEVICE;

/**
 * @ingroup LCI_ENDPOINT
 * @brief Default endpoint initialized by LCI_initialize. Just for convenience.
 */
extern LCI_endpoint_t LCI_UR_ENDPOINT;

/**
 * @ingroup LCI_QUEUE
 * @brief Default completion queue initialized by LCI_initialize. Just for
 * convenience.
 */
extern LCI_comp_t LCI_UR_CQ;

/**
 * @ingroup LCI_SETUP
 * @brief Initialize the LCI runtime. No LCI calls are allowed to be called
 * before LCI_initialize except @ref LCI_initialized.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_initialize();
/**
 * @ingroup LCI_SETUP
 * @brief Check whether the LCI runtime has been initialized
 * @param [in] flag If the runtime has been initialized, it will be set to true.
 * Otherwise, it will be set to false.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_initialized(int* flag);
/**
 * @ingroup LCI_SETUP
 * @brief Finalize the LCI runtime. No LCI calls are allowed to be called
 * after LCI_finalize except @ref LCI_initialized.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_finalize();
/**
 * @ingroup LCI_SETUP
 * @brief Invoke a barrier across all LCI processes in the same job. The call
 * will block the calling thread until the barrier completes. This is not
 * thread-safe.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_barrier();
/**
 * @ingroup LCI_DEVICE
 * @brief Initialize a device.
 * @param [out] device_ptr Pointer to a device to be initialized.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_device_init(LCI_device_t* device_ptr);
LCI_API
LCI_error_t LCI_device_initx(LCI_device_t* device_ptr);
/**
 * @ingroup LCI_DEVICE
 * @brief Initialize a device.
 * @param [in,out] device_ptr Pointer to a device to free.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_device_free(LCI_device_t* device_ptr);
/**
 * @ingroup LCI_PLIST
 * @brief Create a property list.
 * @param [out] plist_ptr Pointer to a property list to create.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_plist_create(LCI_plist_t* plist_ptr);
/**
 * @ingroup LCI_PLIST
 * @brief Free a property list.
 * @param [in,out] plist_ptr Pointer to a property list to free.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_plist_free(LCI_plist_t* plist_ptr);
/**
 * @ingroup LCI_PLIST
 * @brief Query the property list of an endpoint.
 * @param [in]  ep    The endpoint to query.
 * @param [out] plist Pointer to the property list to write the query result.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_plist_get(LCI_endpoint_t ep, LCI_plist_t* plist_ptr);
/**
 * @ingroup LCI_PLIST
 * @brief Decode a property list into a string.
 * @param [in]  plist  The property list to decode.
 * @param [out] string Pointer to an array of chars to write the decode result.
 * @return No available yet. Should always be LCI_ERR_FEATURE_NA.
 */
LCI_API
LCI_error_t LCI_plist_decode(LCI_plist_t plist, char* string);
/**
 * @ingroup LCI_PLIST
 * @brief Set the tag matching rule of the property list.
 * @param [in] plist      The property list to set.
 * @param [in] match_type The matching type to set.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_plist_set_match_type(LCI_plist_t plist, LCI_match_t match_type);
/**
 * @ingroup LCI_PLIST
 * @brief Set the completion mechanism of the property list.
 * @param [in] plist      The property list to set.
 * @param [in] port       Whether to set the completion mechanism for send or
 * receive.
 * @param [in] match_type The matching type to set.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_plist_set_comp_type(LCI_plist_t plist, LCI_port_t port,
                                    LCI_comp_type_t comp_type);
/**
 * @ingroup LCI_PLIST
 * @brief Set the default completion mechanism of the property list. The default
 * completion mechanism will be triggered by a remote one-sided communication
 * operation through LCI_DEFAULT_COMP_REMOTE.
 * @param [in] plist The property list to set.
 * @param [in] comp  The default completion mechanism to set.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_plist_set_default_comp(LCI_plist_t plist, LCI_comp_t comp);
/**
 * @ingroup LCI_ENDPOINT
 * @brief Create an endpoint according to a property list.
 * @param [out] ep_ptr Pointer to the endpoint to create.
 * @param [in]  device The device it should be associated to.
 * @param [in]  plist  The property list to create according to.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_endpoint_init(LCI_endpoint_t* ep_ptr, LCI_device_t device,
                              LCI_plist_t plist);
/**
 * @ingroup LCI_ENDPOINT
 * @brief Free an endpoint.
 * @param [in,out] ep_ptr Pointer to the endpoint to free.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_endpoint_free(LCI_endpoint_t* ep_ptr);
/**
 * @ingroup LCI_COMM
 * @brief Send a short message (up to LCI_SHORT_SIZE bytes). The send buffer
 * can be immediately reused.
 * @param [in] ep   The endpoint to post this send to.
 * @param [in] src  The data to send.
 * @param [in] rank The rank of the destination process.
 * @param [in] tag  The tag of this message.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_sends(LCI_endpoint_t ep, LCI_short_t src, int rank,
                      LCI_tag_t tag);
/**
 * @ingroup LCI_COMM
 * @brief Send a medium message with a user-provided buffer (up to
 * LCI_MEDIUM_SIZE bytes). The send buffer can be reused after completion
 * notification.
 * @param [in] ep     The endpoint to post this send to.
 * @param [in] buffer The buffer to send.
 * @param [in] rank   The rank of the destination process.
 * @param [in] tag    The tag of this message.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_sendmc(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                       LCI_tag_t tag, LCI_comp_t completion,
                       void* user_context);
/**
 * @ingroup LCI_COMM
 * @brief Send a medium message with a user-provided buffer (up to
 * LCI_MEDIUM_SIZE bytes). The send buffer can be immediately reused.
 * @param [in] ep     The endpoint to post this send to.
 * @param [in] buffer The buffer to send.
 * @param [in] rank   The rank of the destination process.
 * @param [in] tag    The tag of this message.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_sendm(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                      LCI_tag_t tag);
/**
 * @ingroup LCI_COMM
 * @brief Send a medium message with a LCI-provided buffer (allocated by
 * @ref LCI_mbuffer_alloc, up to LCI_MEDIUM_SIZE bytes). The send buffer will be
 * directly returned to LCI if send succeeds so users should not use it anymore.
 * @param [in] ep     The endpoint to post this send to.
 * @param [in] buffer The buffer to send. The buffer should be allocated by
 * @ref LCI_mbuffer_alloc.
 * @param [in] rank   The rank of the destination process.
 * @param [in] tag    The tag of this message.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_sendmn(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                       LCI_tag_t tag);
/**
 * @ingroup LCI_COMM
 * @brief Send a long message with a user-provided buffer. The send buffer
 * cannot be reused until the associated completion object marks this
 * operation as completed.
 * @param [in] ep           The endpoint to post this send to.
 * @param [in] buffer       The buffer to send.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_sendl(LCI_endpoint_t ep, LCI_lbuffer_t buffer, int rank,
                      LCI_tag_t tag, LCI_comp_t completion, void* user_context);
/**
 * @ingroup LCI_COMM
 * @brief Receive a short message (up to LCI_SHORT_SIZE bytes). The received
 * message will be delivered through the completion object.
 * @param [in] ep           The endpoint to post this receive to.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_recvs(LCI_endpoint_t ep, int rank, LCI_tag_t tag,
                      LCI_comp_t completion, void* user_context);
/**
 * @ingroup LCI_COMM
 * @brief Receive a medium message (up to LCI_MEDIUM_SIZE bytes) into a
 * user-provided buffer.
 * @param [in] ep           The endpoint to post this receive to.
 * @param [in] buffer       The user-provided buffer to receive the message.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_recvm(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                      LCI_tag_t tag, LCI_comp_t completion, void* user_context);
/**
 * @ingroup LCI_COMM
 * @brief Receive a medium message (up to LCI_MEDIUM_SIZE bytes) into a
 * LCI-provided buffer. The receive buffer will be delivered to users through
 * the completion object and should be returned to the runtime through
 * @ref LCI_mbuffer_free.
 * @param [in] ep           The endpoint to post this receive to.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_recvmn(LCI_endpoint_t ep, int rank, LCI_tag_t tag,
                       LCI_comp_t completion, void* user_context);
/**
 * @ingroup LCI_COMM
 * @brief Receive a long message into a user-provided buffer.
 * @param [in] ep           The endpoint to post this receive to.
 * @param [in] buffer       The user-provided buffer to receive the message.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_recvl(LCI_endpoint_t ep, LCI_lbuffer_t buffer, int rank,
                      LCI_tag_t tag, LCI_comp_t completion, void* user_context);
/**
 * @ingroup LCI_COMM
 * @brief Send a short message (up to LCI_SHORT_SIZE bytes).  The send buffer
 * can be immediately reused. On the receive side, No receive needs to be posted
 * and the data will be delivered through the remote completion object.
 * @param [in] ep           The endpoint to post this receive to.
 * @param [in] src          The data to send.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] remote_completion The completion object to signal on the receiver
 * side. It has to be LCI_DEFAULT_COMP_REMOTE for now.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_puts(LCI_endpoint_t ep, LCI_short_t src, int rank,
                     LCI_tag_t tag, uintptr_t remote_completion);
/**
 * @ingroup LCI_COMM
 * @brief Send a medium message with a user-provided buffer (up to
 * LCI_MEDIUM_SIZE bytes) with completion object.
 * @param [in] ep           The endpoint to post this receive to.
 * @param [in] buffer       The buffer to send.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] remote_completion The completion object to signal on the receiver
 * side. It has to be LCI_DEFAULT_COMP_REMOTE for now.
 * @param [in] local_completion The local completion object to be associated
 * with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_putmac(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                       LCI_tag_t tag, uintptr_t remote_completion,
                       LCI_comp_t local_completion, void* user_context);
/**
 * @ingroup LCI_COMM
 * @brief Send a medium message with a user-provided buffer (up to
 * LCI_MEDIUM_SIZE bytes). The send buffer can be immediately reused. On the
 * receive side, No receive needs to be posted and the data will be delivered
 * through the remote completion object.
 * @param [in] ep           The endpoint to post this receive to.
 * @param [in] buffer       The buffer to send.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] remote_completion The completion object to signal on the receiver
 * side. It has to be LCI_DEFAULT_COMP_REMOTE for now.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_putma(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                      LCI_tag_t tag, uintptr_t remote_completion);
/**
 * @ingroup LCI_COMM
 * @brief Send a medium message with a LCI-provided buffer (allocated by
 * @ref LCI_mbuffer_alloc, up to LCI_MEDIUM_SIZE bytes). The send buffer will be
 * directly returned to LCI if send succeeds so users should not use it anymore.
 * On the receive side, No receive needs to be posted and the data will be
 * delivered through the remote completion object.
 * @param [in] ep           The endpoint to post this receive to.
 * @param [in] buffer       The buffer to send.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] remote_completion The completion object to signal on the receiver
 * side. It has to be LCI_DEFAULT_COMP_REMOTE for now.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_putmna(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int rank,
                       LCI_tag_t tag, uintptr_t remote_completion);
/**
 * @ingroup LCI_COMM
 * @brief Send a long message with a user-provided buffer. The send buffer
 * cannot be reused until the associated completion object marks this
 * operation as completed. On the receive side, No receive needs to be posted
 * and the data will be delivered through the remote completion object.
 * @param [in] ep           The endpoint to post this send to.
 * @param [in] buffer       The buffer to send.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] remote_completion The completion object to signal on the receiver
 * side. It has to be LCI_DEFAULT_COMP_REMOTE for now.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_putla(LCI_endpoint_t ep, LCI_lbuffer_t buffer,
                      LCI_comp_t completion, int rank, LCI_tag_t tag,
                      uintptr_t remote_completion, void* user_context);
/**
 * @ingroup LCI_COMM
 * @brief Send an iovec with user-provided buffers. The piggyback buffer can be
 * immediately reused. The long buffers cannot be reused until the associated
 * completion object marks this operation as completed. On the receive side, No
 * receive needs to be posted and the data will be delivered through the remote
 * completion object.
 * @param [in] ep           The endpoint to post this send to.
 * @param [in] iovec        The iovec to send.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] rank         The rank of the destination process.
 * @param [in] tag          The tag of this message.
 * @param [in] remote_completion The completion object to signal on the receiver
 * side. It has to be LCI_DEFAULT_COMP_REMOTE for now.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @return LCI_OK if the send succeeds. LCI_ERR_RETRY if the send fails due to
 * temporarily unavailable resources. All the other errors are fatal as defined
 * by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_putva(LCI_endpoint_t ep, LCI_iovec_t iovec,
                      LCI_comp_t completion, int rank, LCI_tag_t tag,
                      uintptr_t remote_completion, void* user_context);
/**
 * @ingroup LCI_COMM
 * @brief Return the maximum size of the piggyback buffer in an iovec with given
 * number of long buffers.
 * @param [in] count The number of long buffers in this iovec.
 * @return The maximum size of the piggyback buffer.
 */
LCI_API
size_t LCI_get_iovec_piggy_back_size(int count);
/**
 * @ingroup LCI_QUEUE
 * @brief Create a completion queue.
 *
 * @param [in]  device The device it should be associated to.
 * @param [out] cq     The pointer to the completion queue to create.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_queue_create(LCI_device_t device, LCI_comp_t* cq);
/**
 * @ingroup LCI_QUEUE
 * @brief Create a completion queue with the specified max queue length.
 *
 * @param [in]  device     The device it should be associated to.
 * @param [in]  max_length The maximum length of the queue.
 * @param [out] cq         The pointer to the completion queue to create.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_queue_createx(LCI_device_t device, size_t max_length,
                              LCI_comp_t* cq);
/**
 * @ingroup LCI_QUEUE
 * @brief Free a completion queue.
 *
 * @param [in,out] cq The pointer to the completion queue to free.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_queue_free(LCI_comp_t* cq);
/**
 * @ingroup LCI_QUEUE
 * @brief Pop one entry from the completion queue. This call is nonblocking.
 *
 * @param [in]  cq      The completion queue to pop.
 * @param [out] request The pointer to a request object. The completion
 * information of the completed operation will be written into it.
 * @return LCI_OK if successfully popped one. LCI_ERR_RETRY if there is no entry
 * to be popped in the queue. All the other errors are fatal as defined by
 * @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_queue_pop(LCI_comp_t cq, LCI_request_t* request);
/**
 * @ingroup LCI_QUEUE
 * @brief Pop one entry from the completion queue. This call will block the
 * calling thread until there is an entry to be popped.
 *
 * @param [in]  cq      The completion queue to pop.
 * @param [out] request The pointer to a request object. The completion
 * information of the completed operation will be written into it.
 * @return LCI_OK if successfully popped one. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_queue_wait(LCI_comp_t cq, LCI_request_t* request);
/**
 * @ingroup LCI_QUEUE
 * @brief Pop multiple entry from the completion queue. This call is
 * nonblocking.
 *
 * @param [in]  cq            The completion queue to pop.
 * @param [in]  request_count The maximum entries to be popped.
 * @param [out] request       An array of request objects of length at least
 * `request_count`.Each of the fist `return_count` entries of the array will
 * contain the completion information of one completed operation.
 * @param [out] request_count The number of entries that has been popped.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_queue_pop_multiple(LCI_comp_t cq, size_t request_count,
                                   LCI_request_t* requests,
                                   size_t* return_count);
/**
 * @ingroup LCI_QUEUE
 * @brief Pop multiple entry from the completion queue.This call will block the
 * calling thread until `request_count` entries has been popped.
 *
 * @param [in]  cq            The completion queue to pop.
 * @param [in]  request_count The number of entries to be popped.
 * @param [out] request       An array of request objects of length at least
 * `request_count`.Each entry of the array will contain the completion
 * information of one completed operation.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_queue_wait_multiple(LCI_comp_t cq, size_t request_count,
                                    LCI_request_t* requests);
/**
 * @ingroup LCI_QUEUE
 * @brief Query the number of entries in a completion queue.
 *
 * @param [in]  cq  The completion queue to query.
 * @param [out] len Return the number of entries in the queue.
 * @return No available yet. Should always be LCI_ERR_FEATURE_NA.
 */
LCI_API
LCI_error_t LCI_queue_len(LCI_comp_t cq, size_t* len);

/**
 * @ingroup LCI_SYNC
 * @brief Create a synchronizer.
 *
 * @param [in]  device     The device it should be associated to.
 * @param [in]  threshold  How many asynchronous operations it will be
 * associated to.
 * @param [out] completion The pointer to the synchronizer to create.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_sync_create(LCI_device_t device, int threshold,
                            LCI_comp_t* completion);
/**
 * @ingroup LCI_SYNC
 * @brief Free a synchronizer.
 *
 * @param [in,out] completion The pointer to the synchronizer to free.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_sync_free(LCI_comp_t* completion);
/**
 * @ingroup LCI_SYNC
 * @brief Signal a synchronizer once. A synchronizer needs to be signaled
 * `threshold` time to be considered triggered.
 *
 * @param [in]  completion The synchronizer to be triggered.
 * @param [in]  request    A request containing the completion information of
 * one completed operation.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_sync_signal(LCI_comp_t completion, LCI_request_t request);
/**
 * @ingroup LCI_SYNC
 * @brief Wait for a synchronizer to be triggered. A synchronizer will
 * be triggered if all the associated operations are completed.
 *
 * @param [in]  completion The synchronizer to be waited.
 * @param [out] request    An array of length `threshold` passed to
 * @ref LCI_sync_create. Every entry of the array will contain the completion
 * information of one completed operation.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_sync_wait(LCI_comp_t completion, LCI_request_t request[]);
/**
 * @ingroup LCI_SYNC
 * @brief Test whether a synchronizer has been triggered. A synchronizer will
 * be triggered if all the associated operations are completed.
 *
 * @param [in]  completion The synchronizer to be tested.
 * @param [out] request    An array of length `threshold` passed to
 * @ref LCI_sync_create. Every entry of the array will contain the completion
 * information of one completed operation.
 * @return LCI_OK if successfully popped one. LCI_ERR_RETRY if there is no entry
 * to be popped in the queue. All the other errors are fatal as defined by
 * @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_sync_test(LCI_comp_t completion, LCI_request_t request[]);
/**
 * @ingroup LCI_HANDLER
 * @brief Create an active message handler.
 *
 * @param [in]  device     The device it should be associated to.
 * @param [in]  handler    The function handler to invoke when an operation
 * completes.
 * @param [out] completion The pointer to the completion queue to create.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_handler_create(LCI_device_t device, LCI_handler_t handler,
                               LCI_comp_t* completion);
/**
 * @ingroup LCI_PROGRESS
 * @brief Make progress on the background works of a device. This function
 * is not thread-safe.
 * @param device The device to make progress on.
 * @return LCI_OK if it made progress on something. LCI_ERR_RETRY if there was
 * background work to make progress on. All the other errors are fatal as
 * defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_progress(LCI_device_t device);
/**
 * @ingroup LCI_MEM
 * @brief Register a memory region to a device.
 * @param [in]  device  The device to register.
 * @param [in]  address The starting address of the memory region.
 * @param [in]  length  The length of the memory region.
 * @param [out] segment The registration descriptor.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_memory_register(LCI_device_t device, void* address,
                                size_t length, LCI_segment_t* segment);
/**
 * @ingroup LCI_MEM
 * @brief Deregister a memory region.
 * @param [in,out] segment The registration descriptor to deregister.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_memory_deregister(LCI_segment_t* segment);
/**
 * @ingroup LCI_MEM
 * @brief Get a pre-registered memory buffer of size LCI_MEDIUM_SIZE.
 * @param [in]  device  The device the buffer is registered to.
 * @param [out] mbuffer The pre-registered memory buffer.
 * @return LCI_OK if successfully get one. LCI_ERR_RETRY if buffers are
 * temporarily unavailable. All the other errors are fatal as defined by
 * @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_mbuffer_alloc(LCI_device_t device, LCI_mbuffer_t* mbuffer);
/**
 * @ingroup LCI_MEM
 * @brief Return a pre-registered memory buffer to the LCI runtime.
 * @param [in] mbuffer The pre-registered memory buffer to return.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_mbuffer_free(LCI_mbuffer_t mbuffer);
/**
 * @ingroup LCI_MEM
 * @brief Allocate a memory buffer of arbitrary size and register it.
 * @note This is just a convenient function for now.
 * @param [in]  device  The device to register the buffer.
 * @param [in]  size    The size of the buffer to allocate.
 * @param [out] lbuffer The resulting buffer.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_lbuffer_alloc(LCI_device_t device, size_t size,
                              LCI_lbuffer_t* lbuffer);
/**
 * @ingroup LCI_MEM
 * @brief Allocate a memory buffer of arbitrary size with a specific alignment
 * and register it.
 * @note This is just a convenient function for now.
 * @param [in]  device    The device to register the buffer.
 * @param [in]  size      The size of the buffer to allocate.
 * @param [in]  alignment The alignment of the buffer to allocate.
 * @param [out] lbuffer   The resulting buffer.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_lbuffer_memalign(LCI_device_t device, size_t size,
                                 size_t alignment, LCI_lbuffer_t* lbuffer);
/**
 * @ingroup LCI_MEM
 * @brief Deregister and free a memory buffer.
 * @note This is just a convenient function for now.
 * @param [in] lbuffer The buffer to deregister and free.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCI_lbuffer_free(LCI_lbuffer_t lbuffer);

struct LCIX_collective_s;
/**
 * @ingroup LCI_COLL
 * @brief Handler of a collective operation.
 */
typedef struct LCIX_collective_s* LCIX_collective_t;

/**
 * @ingroup LCI_COLL
 * @brief Make progress on a specified collective operation.
 * @param coll The collective to make progress on.
 * @return LCI_OK if it made progress on something. LCI_ERR_RETRY if there was
 * background work to make progress on. All the other errors are fatal as
 * defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCIX_coll_progress(LCIX_collective_t* coll);

/**
 * @ingroup LCI_COLL
 * @brief Notify all processes that this barrier has been reached. Completion
 * indicates that all processes have reached this barrier.
 * @param [in] ep           The endpoint to post this barrier to.
 * @param [in] tag          The tag of this collective. This should be the same
 * at all processes participating in this collective and distinct from all
 * other communications concurrently executing on this endpoint.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @param [out] coll        The collective progress structure.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCIX_barrier(LCI_endpoint_t ep, LCI_tag_t tag,
                         LCI_comp_t completion, void* user_context,
                         LCIX_collective_t* coll);

/**
 * @ingroup LCI_COLL
 * @brief Broadcast a medium message (up to LCI_MEDIUM_SIZE bytes) with a
 * user-provided buffer from root to all processes. The buffer cannot be reused
 * until the associated completion object marks this operation as completed.
 * @param [in] ep           The endpoint to post this broadcast to.
 * @param [in] buffer       The user-provided buffer to communicate the message.
 * This will be the send buffer at the root and the receive buffer at all other
 * processes. The length of the buffer should be the same at all processes.
 * @param [in] root         The root of the broadcast. This should be the same
 * on all processes participating in this broadcast.
 * @param [in] tag          The tag of this collective. This should be the same
 * at all processes participating in this collective and distinct from all
 * other communications concurrently executing on this endpoint.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @param [out] coll        The collective progress structure.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCIX_bcastm(LCI_endpoint_t ep, LCI_mbuffer_t buffer, int root,
                        LCI_tag_t tag, LCI_comp_t completion,
                        void* user_context, LCIX_collective_t* coll);

/**
 * @ingroup LCI_COLL
 * @brief Broadcast a long message with a user-provided buffer from root to all
 * processes. The buffer cannot be reused until the associated completion
 * object marks this operation as completed.
 * @param [in] ep           The endpoint to post this broadcast to.
 * @param [in] buffer       The user-provided buffer to communicate the message.
 * This will be the send buffer at the root and the receive buffer at all other
 * processes. The length of the buffer should be the same at all processes.
 * @param [in] root         The root of the broadcast.
 * @param [in] tag          The tag of this collective. This should be the same
 * at all processes participating in this collective and distinct from all
 * other communications concurrently executing on this endpoint.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @param [out] coll        The collective progress structure.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCIX_bcastl(LCI_endpoint_t ep, LCI_lbuffer_t buffer, int root,
                        LCI_tag_t tag, LCI_comp_t completion,
                        void* user_context, LCIX_collective_t* coll);

/**
 * @ingroup LCI_COLL
 * @brief The reduction operation type. It is assumed to be commutative.
 * @param [in,out] dest   The buffer containing the current reduction output.
 * @param [in]     src    The buffer containing new input to reduce.
 * @param [in]     length The size of both buffers, in bytes.
 */
typedef void (*LCI_op_t)(void* dest, const void* src, size_t length);

/**
 * @ingroup LCI_COLL
 * @brief Reduce a medium message (up to LCI_MEDIUM_SIZE bytes) with a
 * user-provided buffer across all processes. The buffer cannot be reused
 * until the associated completion object marks this operation as completed.
 * @param [in] ep           The endpoint to post this reduction to.
 * @param [in] buffer       The user-provided buffer to communicate the message.
 * The length of the buffer should be the same at all processes.
 * @param [in] tag          The tag of this collective. This should be the same
 * at all processes participating in this collective and distinct from all
 * other communications concurrently executing on this endpoint.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @param [out] coll        The collective progress structure.
 * @return LCI_OK if successfully started. LCI_ERR_RETRY if buffers are
 * temporarily unavailable. All the other errors are fatal as defined by
 * @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCIX_allreducem(LCI_endpoint_t ep, LCI_mbuffer_t buffer,
                            LCI_op_t op, LCI_tag_t tag, LCI_comp_t completion,
                            void* user_context, LCIX_collective_t* coll);

/**
 * @ingroup LCI_COLL
 * @brief Reduce a long message with a user-provided buffer across all
 * processes. The buffer cannot be reused until the associated completion
 * object marks this operation as completed.
 * @param [in] ep           The endpoint to post this reduction to.
 * @param [in] buffer       The user-provided buffer to communicate the message.
 * The length of the buffer should be the same at all processes.
 * @param [in] tag          The tag of this collective. This should be the same
 * at all processes participating in this collective and distinct from all
 * other communications concurrently executing on this endpoint.
 * @param [in] completion   The completion object to be associated with.
 * @param [in] user_context Arbitrary data the user want to attach to this
 * operation. It will be returned the user through the completion object.
 * @param [out] coll        The collective progress structure.
 * @return Should always be LCI_OK. All the other errors are fatal
 * as defined by @ref LCI_error_t.
 */
LCI_API
LCI_error_t LCIX_allreducel(LCI_endpoint_t ep, LCI_lbuffer_t buffer,
                            LCI_op_t op, LCI_tag_t tag, LCI_comp_t completion,
                            void* user_context, LCIX_collective_t* coll);

#ifdef __cplusplus
}
#endif

#endif
