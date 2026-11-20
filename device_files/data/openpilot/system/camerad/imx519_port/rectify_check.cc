#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#include <cassert>
#include <cstdio>
#include <cstring>
#include "common/util.h"
#include "common/timing.h"
#include "system/camerad/imx519_port/cameras/imx519_rectify.h"

int main(int argc,char **argv) {
  assert(argc==3);
  const auto device=cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  const auto ctx=CL_CHECK_ERR(clCreateContext(nullptr,1,&device,nullptr,nullptr,&err));
  const auto raw=util::read_file(argv[1]);
  assert(raw.size()==1402880);
  VisionBuf src,dst;
  for(auto *b:{&src,&dst}) {b->allocate(raw.size());b->init_yuv(1280,720,1280,942080);b->init_cl(device,ctx);}
  memcpy(src.addr,raw.data(),raw.size());src.sync(VISIONBUF_SYNC_TO_DEVICE);
  {
    Imx519Rectifier rectifier(ctx,device);
    double elapsed=0;
    for(int i=0;i<31;++i) {
      const auto begin=nanos_since_boot();rectifier.run(&src,&dst);
      if(i)elapsed+=(nanos_since_boot()-begin)/1e6;
    }
    fprintf(stderr,"rectify mean ms %.3f\n",elapsed/30);
    dst.sync(VISIONBUF_SYNC_FROM_DEVICE);
    FILE *f=fopen(argv[2],"wb");assert(f);assert(fwrite(dst.addr,1,raw.size(),f)==raw.size());fclose(f);
  }
  src.free();dst.free();clReleaseContext(ctx);
}
