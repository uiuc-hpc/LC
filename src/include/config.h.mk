#ifndef CONFIG_H_
#define CONFIG_H_

// NOTE(danghvu): These numbers are tweaked for performance and some alignment.
// Update at our own risk.

#ifdef LC_USE_SERVER_OFI
#define LC_PACKET_SIZE (32 * 1024 + 4096)
#else
#define LC_PACKET_SIZE (8 * 1024 + 4096)
#endif  // LC_USE_SERVER_OFI

#define LC_MAX_INLINE 128
#define LC_DEV_MEM_SIZE (8*1024*1024)

// Using LCRQ or spinlock.
// #define USE_CCQ

#define LC_SERVER_MAX_RCVS 64
#define LC_SERVER_NUM_PKTS 1024
#define LC_CACHE_LINE 64

#endif
