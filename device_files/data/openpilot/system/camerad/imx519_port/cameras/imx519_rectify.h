#pragma once
#include <cassert>

#include "common/clutil.h"
#include "msgq/visionipc/visionbuf.h"

// Profile: auto_20261029_045412, 42 views, fixed SA3103 DAC 1366.
// Output retains the measured K and dimensions, with no crop or zoom.
// Input is the original IFE NV12 image. Never rectify an already rectified frame.
static const char imx519_rectify_source[] = R"CLC(
inline float2 lens_map(float2 p) {
  const float fx=1181.1035921673f, fy=1180.8832755333f;
  const float cx=663.8852284771f, cy=307.6199600165f;
  float x=(p.x-cx)/fx, y=(p.y-cy)/fy, r2=x*x+y*y;
  float radial=1.f+r2*(.17133582498155f+r2*(-1.3754921710514f+r2*3.0988606658689f));
  float xd=x*radial+2.f*(-.00023088981041f)*x*y+(-.00083198331489f)*(r2+2.f*x*x);
  float yd=y*radial+(-.00023088981041f)*(r2+2.f*y*y)+2.f*(-.00083198331489f)*x*y;
  return (float2)(fx*xd+cx,fy*yd+cy);
}
inline uchar sample_plane(__global const uchar *src, float2 p, int stride,
                          int offset, int width, int height, int step, uchar border) {
  if(p.x<0.f || p.y<0.f || p.x>width-1 || p.y>height-1) return border;
  int x=(int)floor(p.x), y=(int)floor(p.y);
  int xp=min(x+1,width-1), yp=min(y+1,height-1);
  float ax=p.x-x, ay=p.y-y;
  float v=mix(mix((float)src[offset+y*stride+x*step],(float)src[offset+y*stride+xp*step],ax),
              mix((float)src[offset+yp*stride+x*step],(float)src[offset+yp*stride+xp*step],ax),ay);
  return convert_uchar_sat_rte(v);
}
__kernel void rectify_nv12(__global const uchar *src,__global uchar *dst,int stride,int uv_offset) {
  int bx=get_global_id(0), by=get_global_id(1);
  if(bx>=640 || by>=360)return;
  int x=bx*2,y=by*2;
  for(int dy=0;dy<2;++dy)for(int dx=0;dx<2;++dx)
    dst[(y+dy)*stride+x+dx]=sample_plane(src,lens_map((float2)(x+dx,y+dy)),stride,0,1280,720,1,16);
  float2 uv=(lens_map((float2)(x+.5f,y+.5f))-.5f)*.5f;
  dst[uv_offset+by*stride+x]=sample_plane(src,uv,stride,uv_offset,640,360,2,128);
  dst[uv_offset+by*stride+x+1]=sample_plane(src,uv,stride,uv_offset+1,640,360,2,128);
}
)CLC";

class Imx519Rectifier {
  cl_command_queue queue=nullptr;
  cl_program program=nullptr;
  cl_kernel kernel=nullptr;
public:
  Imx519Rectifier(cl_context ctx,cl_device_id device) {
    queue=CL_CHECK_ERR(clCreateCommandQueue(ctx,device,0,&err));
    program=cl_program_from_source(ctx,device,imx519_rectify_source,nullptr);
    kernel=CL_CHECK_ERR(clCreateKernel(program,"rectify_nv12",&err));
  }
  ~Imx519Rectifier() {
    if(kernel)clReleaseKernel(kernel);
    if(program)clReleaseProgram(program);
    if(queue)clReleaseCommandQueue(queue);
  }
  void run(VisionBuf *src,VisionBuf *dst) {
    assert(src!=dst && src->width==1280 && src->height==720);
    assert(src->stride==dst->stride && src->uv_offset==dst->uv_offset);
    const int stride=src->stride,uv=src->uv_offset;
    CL_CHECK(clSetKernelArg(kernel,0,sizeof(cl_mem),&src->buf_cl));
    CL_CHECK(clSetKernelArg(kernel,1,sizeof(cl_mem),&dst->buf_cl));
    CL_CHECK(clSetKernelArg(kernel,2,sizeof(int),&stride));
    CL_CHECK(clSetKernelArg(kernel,3,sizeof(int),&uv));
    const size_t work[]={640,360};
    CL_CHECK(clEnqueueNDRangeKernel(queue,kernel,2,nullptr,work,nullptr,0,nullptr,nullptr));
    CL_CHECK(clFinish(queue));
  }
};
