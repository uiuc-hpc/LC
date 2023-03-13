#ifndef COMM_EXP_H_
#define COMM_EXP_H_

#include <sys/time.h>

#define MIN_MSG (1)
#define MAX_MSG (4 * 1024 * 1024)

#define LARGE 8192

#define NEXP 10

#define TOTAL 10000
#define SKIP 1000

#define TOTAL_LARGE 1000
#define SKIP_LARGE 100

#define DEFAULT_NUM_WORKER 4
#define DEFAULT_NUM_THREAD 4

static inline double wtime(void)
{
  struct timeval t1;
  gettimeofday(&t1, 0);
  return t1.tv_sec + t1.tv_usec / 1e6;
}

static inline double wutime(void)
{
  struct timeval t1;
  gettimeofday(&t1, 0);
  return t1.tv_sec * 1e6 + t1.tv_usec;
}

#define max(a, b) ((a > b) ? (a) : (b))

static inline unsigned long long get_rdtsc(void)
{
  unsigned hi, lo;
  __asm__ __volatile__("rdtsc" : "=a"(lo), "=d"(hi));
  unsigned long long cycle =
      ((unsigned long long)lo) | (((unsigned long long)hi) << 32);
  return cycle;
}

static inline void busywait_cyc(unsigned long long delay)
{
  unsigned long long start_cycle, stop_cycle, start_plus_delay;
  start_cycle = get_rdtsc();
  start_plus_delay = start_cycle + delay;
  do {
    stop_cycle = get_rdtsc();
  } while (stop_cycle < start_plus_delay);
}

#ifdef __cplusplus
#include <mutex>
#include <condition_variable>
class cpp_barrier
{
 private:
  std::mutex _mutex;
  std::condition_variable _cv;
  std::size_t _count;

 public:
  explicit cpp_barrier(std::size_t count) : _count{count} {}
  void wait()
  {
    std::unique_lock<std::mutex> lock{_mutex};
    if (--_count == 0) {
      _cv.notify_all();
    } else {
      _cv.wait(lock, [this] { return _count == 0; });
    }
  }
};
#endif
#endif
