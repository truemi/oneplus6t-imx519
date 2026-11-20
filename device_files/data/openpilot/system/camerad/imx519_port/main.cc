#include <execinfo.h>
#include <signal.h>
#include "system/camerad/imx519_port/cameras/camera_common.h"

#include <cassert>
#include <cerrno>
#include <cstring>
#include <sched.h>
#include <unistd.h>

#include "common/params.h"
#include "common/util.h"

// Pin current process to big core 6 (Kryo 385 Gold / A75)
// Retries on failure — launcher fork timing can cause transient EINVAL/ESRCH
static void pin_to_big_core() {
  for (int attempt = 0; attempt < 3; attempt++) {
    int ret = util::set_core_affinity({6});
    if (ret == 0) {
      fprintf(stderr, "set_core_affinity({6}) OK (attempt %d)\n", attempt);
      return;
    }
    int err = errno;
    fprintf(stderr, "set_core_affinity({6}) failed attempt %d: ret=%d errno=%d (%s)\n",
            attempt, ret, err, strerror(err));

    // Try alternative: sched_setaffinity with pid=0 (current thread shorthand)
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(6, &cpus);
    if (sched_setaffinity(0, sizeof(cpus), &cpus) == 0) {
      fprintf(stderr, "sched_setaffinity(0, {6}) fallback OK\n");
      return;
    }
    err = errno;
    fprintf(stderr, "sched_setaffinity(0, {6}) fallback failed: errno=%d (%s)\n", err, strerror(err));

    usleep(100000);  // 100ms before retry
  }
  fprintf(stderr, "WARNING: could not pin to core 6 after retries, continuing on default cores\n");
}

int main(int argc, char *argv[]) {
  const bool bench=getenv("IMX519_DIRECT_BENCH")!=nullptr;
  const bool active=getenv("IMX519_NATIVE_ACTIVE")!=nullptr;
  if(!bench && !active) { fprintf(stderr,"Explicit IMX519 backend mode required\n"); return 2; }
  if(active) {
    FILE *f=fopen("/data/imx519_native_enabled","rb"); int state=f?fgetc(f):-1; if(f)fclose(f);
    if(state!='1') return 2;
    if(!getenv("IMX519_RECTIFY") || !getenv("IMX519_OUTPUT_720")) return 2;
    FILE *g=fopen("/data/imx519_geometry_enabled","rb"); int geometry=g?fgetc(g):-1; if(g)fclose(g);
    if(geometry!='1') return 2;
  }

  // A private calibration/probe remains parked-only. Production capture follows
  // manager lifetime and must survive the transition to onroad or engagement.
  if(bench) for (const char *key : {"IsOnroad", "IsEngaged"}) {
    std::string path=std::string("/data/params/d/")+key;
    FILE *f=fopen(path.c_str(),"rb"); int state=f?fgetc(f):-1; if(f) fclose(f);
    if(state!='0') { fprintf(stderr,"Device must be parked and disengaged\n"); return 3; }
  }
  signal(SIGSEGV, [](int){ void *bt[32]; int n=backtrace(bt,32); backtrace_symbols_fd(bt,n,2); _exit(139); });
  setvbuf(stdout,nullptr,_IONBF,0);
  pin_to_big_core();

  camerad_thread();
  return 0;
}
