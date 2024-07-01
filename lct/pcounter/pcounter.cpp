#include <utility>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <stdexcept>
#include "lcti.hpp"

namespace lct::pcounter
{
struct record_t;
struct ctx_t;

// magic number
const int naccesses_per_record = 1000;

struct entry_t {
  entry_t() : total(0), count(0), min(INT64_MAX), max(INT64_MIN) {}
  void add(int64_t val)
  {
    total += val;
    ++count;
    min = val < min ? val : min;
    max = val > max ? val : max;
  }
  void merge(entry_t other)
  {
    total += other.total;
    count += other.count;
    min = other.min < min ? other.min : min;
    max = other.max > max ? other.max : max;
  }

  int64_t total;
  int64_t count;
  int64_t min;
  int64_t max;
};

struct record_t {
  explicit record_t(LCT_time_t time_, const std::vector<entry_t>& entries_)
      : time(time_), entries(entries_)
  {
  }
  LCT_time_t time;
  std::vector<entry_t> entries;
};

struct timer_t {
  timer_t() : consecutive_start(false), start_time(0), start_count(0) {}
  bool start(LCT_time_t time)
  {
    if (start_count != 0) consecutive_start = true;
    start_time += time;
    ++start_count;
    return true;
  }
  void end(LCT_time_t time)
  {
    entry.add(time - start_time);
    start_time = 0;
    --start_count;
  }
  void add(LCT_time_t time) { entry.add(time); }
  [[nodiscard]] entry_t get() const
  {
    entry_t ret = entry;
    if (consecutive_start) {
      // min and max is not valid
      ret.min = 1;
      ret.max = 0;
    }
    //    else {
    //      ret.min = static_cast<int64_t>(LCT_time_to_ns(ret.min));
    //      ret.max = static_cast<int64_t>(LCT_time_to_ns(ret.max));
    //    }
    //    ret.total = static_cast<int64_t>(LCT_time_to_ns(ret.total));
    // Here we exploit the fact that LCT_time_to_ns is an identity function.
    return ret;
  }
  bool consecutive_start;
  LCT_time_t start_time;
  int64_t start_count;
  entry_t entry;
};

struct ctx_t;

struct tls_ctx_t {
  explicit tls_ctx_t(std::string name_,
                     const std::vector<std::string>& counter_names_,
                     const std::vector<std::string>& trend_names_,
                     const std::vector<std::string>& timer_names_)
      : name(std::move(name_)),
        counters(counter_names_.size()),
        trends(trend_names_.size()),
        timers(timer_names_.size()),
        counter_names(counter_names_),
        trend_names(trend_names_),
        timer_names(timer_names_)
  {
  }

  void add(LCT_pcounter_handle_t handle, int64_t val)
  {
    lock.lock();
    switch (handle.type) {
      case LCT_PCOUNTER_COUNTER:
        if (handle.idx >= counters.size()) {
          counters.resize(handle.idx + 1);
        }
        counters[handle.idx].add(val);
        break;
      case LCT_PCOUNTER_TREND:
        if (handle.idx >= trends.size()) {
          trends.resize(handle.idx + 1);
        }
        trends[handle.idx].add(val);
        break;
      case LCT_PCOUNTER_TIMER:
        if (handle.idx >= timers.size()) {
          timers.resize(handle.idx + 1);
        }
        timers[handle.idx].add(val);
        break;
      default:
        throw std::runtime_error("add: unexpected type! " + name +
                                 ", type: " + std::to_string(handle.type) +
                                 ", idx: " + std::to_string(handle.idx));
    }
    lock.unlock();
  }

  void start(LCT_pcounter_handle_t handle, LCT_time_t time)
  {
    lock.lock();
    if (handle.type != LCT_PCOUNTER_TIMER)
      throw std::runtime_error("start: unexpected type! " + name +
                               ", type: " + std::to_string(handle.type) + ", " +
                               std::to_string(handle.idx));
    if (handle.idx >= timers.size()) {
      timers.resize(handle.idx + 1);
    }
    timers[handle.idx].start(time);
    lock.unlock();
  }

  void end(LCT_pcounter_handle_t handle, LCT_time_t time)
  {
    lock.lock();
    if (handle.type != LCT_PCOUNTER_TIMER)
      throw std::runtime_error("end: unexpected type! " + name + " " +
                               std::to_string(handle.type) + " " +
                               std::to_string(handle.idx));
    if (handle.idx >= timers.size()) {
      timers.resize(handle.idx + 1);
    }
    timers[handle.idx].end(time);
    lock.unlock();
  }

  std::vector<entry_t> get_counters()
  {
    lock.lock();
    auto ret = counters;
    lock.unlock();
    return ret;
  }

  std::vector<entry_t> get_trends()
  {
    lock.lock();
    auto ret = trends;
    lock.unlock();
    return ret;
  }

  std::vector<timer_t> get_timers()
  {
    lock.lock();
    auto ret = timers;
    lock.unlock();
    return ret;
  }

  std::vector<entry_t> counters;
  std::vector<entry_t> trends;
  std::vector<timer_t> timers;
  std::vector<std::string> counter_names;
  std::vector<std::string> trend_names;
  std::vector<std::string> timer_names;
  spinlock_t lock;
  std::string name;
};

thread_local std::vector<tls_ctx_t*> tls_ctxs;

void record_thread_fn(ctx_t* ctx, uint64_t record_interval);

struct ctx_t {
  explicit ctx_t(const char* name_)
      : dump_ofile(nullptr),
        dump_record_on_the_fly(false),
        dump_record_on_the_fly_lw(false),
        name(name_),
        id(next_id++),
        record_thread(nullptr),
        do_record(false),
        total_record_time(0),
        total_initialize_time(0)
  {
    std::string dump_ofilename;
    uint64_t record_interval = 0;
    // Pre-defined mode for convenience
    {
      std::string mode_str;
      char* p = getenv("LCT_PCOUNTER_MODE");
      if (p) {
        mode_str = p;
      }
      if (mode_str == "on-the-fly") {
        dump_ofilename = std::string("lct_pcounter.") + LCT_hostname + ".%.out";
        dump_record_on_the_fly = true;
        record_interval = 1000000;
      } else if (mode_str == "on-the-fly-lw") {
        dump_ofilename = std::string("lct_pcounter.") + LCT_hostname + ".%.out";
        dump_record_on_the_fly = true;
        dump_record_on_the_fly_lw = true;
        record_interval = 1000000;
      }
    }
    // Output file to dump
    {
      char* p = getenv("LCT_PCOUNTER_AUTO_DUMP");
      if (p) dump_ofilename = p;
    }
    if (!dump_ofilename.empty()) {
      if (dump_ofilename == "stderr")
        dump_ofile = stderr;
      else if (dump_ofilename == "stdout")
        dump_ofile = stdout;
      else {
        std::string ofilename =
            replaceOne(dump_ofilename, "%", std::to_string(LCT_get_rank()));
        dump_ofile = fopen(ofilename.c_str(), "a");
        if (dump_ofile == nullptr) {
          fprintf(stderr, "Cannot open the logfile %s!\n", ofilename.c_str());
        }
      }
    }
    // Record interval
    if (dump_ofile && record_interval == 0) {
      record_interval = 1000000;  // by default 1s.
    }
    {
      char* p = getenv("LCT_PCOUNTER_RECORD_INTERVAL");
      if (p) {
        record_interval = std::stoull(p);
      }
    }
    if (record_interval > 0) {
      keep_recording = true;
      record_thread = new std::thread(record_thread_fn, this, record_interval);
    }
    // Whether to dump on the fly
    if (getenv("LCT_PCOUNTER_DUMP_ON_THE_FLY")) {
      dump_record_on_the_fly = true;
    }
    if (getenv("LCT_PCOUNTER_DUMP_ON_THE_FLY_LW")) {
      dump_record_on_the_fly = true;
      dump_record_on_the_fly_lw = true;
    }
    // For now, we just assume there will only be a single thread initializing.
    if (start_time == -1) {
      start_time = LCT_now();
    }
  }
  ~ctx_t()
  {
    if (record_thread) {
      keep_recording = false;
      record_thread->join();
    }
    record();
    if (dump_ofile) dump(dump_ofile);
    if (dump_ofile && dump_ofile != stdout && dump_ofile != stderr)
      fclose(dump_ofile);
    for (auto thread_ctx : thread_ctxs) {
      delete thread_ctx;
    }
    thread_ctxs.clear();
  }

  int register_counter(const char* name_, LCT_pcounter_type_t type)
  {
    int ret;
    switch (type) {
      case LCT_PCOUNTER_COUNTER:
        ret = static_cast<int>(counter_names.size());
        counter_names.emplace_back(name_);
        break;
      case LCT_PCOUNTER_TREND:
        ret = static_cast<int>(trend_names.size());
        trend_names.emplace_back(name_);
        break;
      case LCT_PCOUNTER_TIMER:
        ret = static_cast<int>(timer_names.size());
        timer_names.emplace_back(name_);
        break;
    }
    return ret;
  }

  void initialize_tls_if_necessary()
  {
    if (LCT_unlikely(id >= tls_ctxs.size() || tls_ctxs[id] == nullptr)) {
      auto start = LCT_now();
      // we need to allocate a new tls_ctx
      auto* tls_ctx_p =
          new tls_ctx_t(name, counter_names, trend_names, timer_names);
      if (id >= tls_ctxs.size()) tls_ctxs.resize(id + 1);
      tls_ctxs[id] = tls_ctx_p;

      lock.lock();
      int thread_id = LCT_get_thread_id();
      if (thread_id >= thread_ctxs.size()) {
        thread_ctxs.resize(thread_id + 1, nullptr);
      }
      thread_ctxs[thread_id] = tls_ctx_p;
      lock.unlock();
      total_initialize_time += LCT_now() - start;
    }
  }

  void add(LCT_pcounter_handle_t handle, int64_t val)
  {
    if (handle.type == LCT_PCOUNTER_NONE) return;
    initialize_tls_if_necessary();
    tls_ctxs[id]->add(handle, val);
    // check whether to record
    if (handle.type == LCT_PCOUNTER_TREND && do_record) {
      bool expected = true;
      if (do_record.compare_exchange_weak(expected, false)) {
        record();
        if (dump_record_on_the_fly_lw) dump(dump_ofile);
      }
    }
  }

  void start(LCT_pcounter_handle_t handle, LCT_time_t time)
  {
    if (handle.type == LCT_PCOUNTER_NONE) return;
    initialize_tls_if_necessary();
    tls_ctxs[id]->start(handle, time);
  }

  void end(LCT_pcounter_handle_t handle, LCT_time_t time)
  {
    if (handle.type == LCT_PCOUNTER_NONE) return;
    initialize_tls_if_necessary();
    tls_ctxs[id]->end(handle, time);
  }

  void record()
  {
    lock.lock();
    auto start_record = LCT_now();
    std::vector<entry_t> entries;
    entries.resize(trend_names.size());
    for (const auto& thread_ctx : thread_ctxs) {
      if (thread_ctx == nullptr) continue;
      auto tls_entries = thread_ctx->get_trends();
      for (int i = 0; i < tls_entries.size(); ++i) {
        entries[i].merge(tls_entries[i]);
      }
    }
    records.emplace_back(start_record, entries);
    total_record_time += LCT_now() - start_record;
    lock.unlock();
  }

  void print_entry(FILE* out, const std::string& type_name,
                   LCT_time_t record_time, const std::string& entry_name,
                   const entry_t& entry) const
  {
    if (entry.count > 0) {
      fprintf(out, "pcounter,%s,%d,%s,%ld,%s,%ld,%ld,%ld,%ld,%ld\n",
              type_name.c_str(), LCT_get_rank(), name.c_str(),
              static_cast<int64_t>(LCT_time_to_us(record_time - start_time)),
              entry_name.c_str(), entry.total, entry.count,
              entry.total / entry.count, entry.min, entry.max);
    } else {
      fprintf(out, "pcounter,%s,%d,%s,%ld,%s,,0,,\n", type_name.c_str(),
              LCT_get_rank(), name.c_str(),
              static_cast<int64_t>(LCT_time_to_us(record_time - start_time)),
              entry_name.c_str());
    }
  }

  void dump(FILE* out)
  {
    lock.lock();
    fprintf(out,
            "pcounter-summary: rank %d, name %s, "
            "total records %lu, total record time %ld ns, "
            "total init time %ld ns\n",
            LCT_get_rank(), name.c_str(), records.size(),
            static_cast<int64_t>(LCT_time_to_ns(total_record_time)),
            static_cast<int64_t>(LCT_time_to_ns(total_initialize_time)));
    // dump all records
    for (const auto& record : records) {
      for (int i = 0; i < record.entries.size(); ++i) {
        print_entry(out, "trend", record.time, trend_names[i],
                    record.entries[i]);
      }
    }
    records.clear();
    // dump all counters and timers
    std::vector<entry_t> counters;
    std::vector<entry_t> timers;
    counters.resize(counter_names.size());
    timers.resize(timer_names.size());
    for (const auto& thread_ctx : thread_ctxs) {
      if (thread_ctx == nullptr) continue;
      auto tls_counters = thread_ctx->get_counters();
      for (int i = 0; i < tls_counters.size(); ++i) {
        counters[i].merge(tls_counters[i]);
      }
      auto tls_timers = thread_ctx->get_timers();
      for (int i = 0; i < tls_timers.size(); ++i) {
        timers[i].merge(tls_timers[i].get());
      }
    }
    LCT_time_t now = LCT_now();
    for (int i = 0; i < counters.size(); ++i) {
      print_entry(out, "counter", now, counter_names[i], counters[i]);
    }
    for (int i = 0; i < timers.size(); ++i) {
      print_entry(out, "timer", now, timer_names[i], timers[i]);
    }
    lock.unlock();
  }

  FILE* dump_ofile;
  bool dump_record_on_the_fly;
  bool dump_record_on_the_fly_lw;
  std::vector<std::string> counter_names;
  std::vector<std::string> trend_names;
  std::vector<std::string> timer_names;
  std::vector<tls_ctx_t*> thread_ctxs;
  std::vector<record_t> records;
  LCT_time_t total_record_time;
  LCT_time_t total_initialize_time;
  spinlock_t lock;
  std::string name;
  char padding0[LCT_CACHE_LINE];
  std::atomic<bool> do_record;
  char padding1[LCT_CACHE_LINE];
  std::atomic<bool> keep_recording{};
  std::thread* record_thread;
  int id;
  static std::atomic<int> next_id;
  static LCT_time_t start_time;
};

void record_thread_fn(ctx_t* ctx, uint64_t record_interval)
{
  LCT_Log(LCT_log_ctx_default, LCT_LOG_INFO, "pcounter",
          "recording thread init\n");
  while (ctx->keep_recording) {
    if (ctx->dump_record_on_the_fly_lw)
      ctx->do_record = true;
    else if (ctx->dump_record_on_the_fly) {
      ctx->record();
      ctx->dump(ctx->dump_ofile);
    } else {
      break;
    }
    usleep(record_interval);
  }
  LCT_Log(LCT_log_ctx_default, LCT_LOG_INFO, "pcounter",
          "recording thread exit\n");
}
std::atomic<int> ctx_t::next_id(0);
LCT_time_t ctx_t::start_time = -1;
}  // namespace lct::pcounter

LCT_pcounter_ctx_t LCT_pcounter_ctx_alloc(const char* ctx_name)
{
  auto* ctx = new lct::pcounter::ctx_t(ctx_name);
  return ctx;
}

void LCT_pcounter_ctx_free(LCT_pcounter_ctx_t* pcounter_ctx)
{
  auto* ctx = static_cast<lct::pcounter::ctx_t*>(*pcounter_ctx);
  delete ctx;
  *pcounter_ctx = nullptr;
}

LCT_pcounter_handle_t LCT_pcounter_register(LCT_pcounter_ctx_t pcounter_ctx,
                                            const char* name,
                                            LCT_pcounter_type_t type)
{
  auto* ctx = static_cast<lct::pcounter::ctx_t*>(pcounter_ctx);
  return {type, ctx->register_counter(name, type)};
}

void LCT_pcounter_add(LCT_pcounter_ctx_t pcounter_ctx,
                      LCT_pcounter_handle_t handle, int64_t val)
{
  auto* ctx = static_cast<lct::pcounter::ctx_t*>(pcounter_ctx);
  ctx->add(handle, val);
}

void LCT_pcounter_start(LCT_pcounter_ctx_t pcounter_ctx,
                        LCT_pcounter_handle_t handle)
{
  auto* ctx = static_cast<lct::pcounter::ctx_t*>(pcounter_ctx);
  ctx->start(handle, LCT_now());
}

void LCT_pcounter_end(LCT_pcounter_ctx_t pcounter_ctx,
                      LCT_pcounter_handle_t handle)
{
  auto* ctx = static_cast<lct::pcounter::ctx_t*>(pcounter_ctx);
  ctx->end(handle, LCT_now());
}

void LCT_pcounter_startt(LCT_pcounter_ctx_t pcounter_ctx,
                         LCT_pcounter_handle_t handle, LCT_time_t time)
{
  auto* ctx = static_cast<lct::pcounter::ctx_t*>(pcounter_ctx);
  ctx->start(handle, time);
}

void LCT_pcounter_endt(LCT_pcounter_ctx_t pcounter_ctx,
                       LCT_pcounter_handle_t handle, LCT_time_t time)
{
  auto* ctx = static_cast<lct::pcounter::ctx_t*>(pcounter_ctx);
  ctx->end(handle, time);
}

void LCT_pcounter_record(LCT_pcounter_ctx_t pcounter_ctx)
{
  auto* ctx = static_cast<lct::pcounter::ctx_t*>(pcounter_ctx);
  ctx->record();
}

void LCT_pcounter_dump(LCT_pcounter_ctx_t pcounter_ctx, FILE* out)
{
  auto* ctx = static_cast<lct::pcounter::ctx_t*>(pcounter_ctx);
  ctx->dump(out);
}