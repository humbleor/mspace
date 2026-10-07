// Immutable world-space surfel BVH. Each query uses the current ray origin/direction.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <memory>
#include <vector>
struct Vec { double x[3]; };
struct Surfel { Vec p,n; double radius; int kind; };
struct Node { Vec lo,hi; int left=-1,right=-1,begin=0,end=0; };
struct BVH {
 std::vector<Surfel> surfels; std::vector<int> ids; std::vector<Node> nodes;
 int build(int begin,int end) {
  Node node; node.begin=begin;node.end=end;
  for(int a=0;a<3;++a) { node.lo.x[a]=INFINITY;node.hi.x[a]=-INFINITY; }
  for(int i=begin;i<end;++i) for(int a=0;a<3;++a) {
   double c=surfels[ids[i]].p.x[a]; double radius=surfels[ids[i]].radius;
   node.lo.x[a]=std::min(node.lo.x[a],c-radius);
   node.hi.x[a]=std::max(node.hi.x[a],c+radius);
  }
  int index=static_cast<int>(nodes.size());nodes.push_back(node);
  if(end-begin>8) {
   int axis=0;for(int a=1;a<3;++a) if(node.hi.x[a]-node.lo.x[a]>node.hi.x[axis]-node.lo.x[axis]) axis=a;
   int middle=(begin+end)/2;
   std::nth_element(ids.begin()+begin,ids.begin()+middle,ids.begin()+end,
    [&](int i,int j){return surfels[i].p.x[axis]<surfels[j].p.x[axis];});
   int left=build(begin,middle),right=build(middle,end);
   nodes[index].left=left;nodes[index].right=right;
  }
  return index;
 }
};
static bool box(const Node& n,const double* o,const double* d,double blind,double best,double& near) {
 double far=best;near=blind;
 for(int a=0;a<3;++a) {
  if(std::abs(d[a])<1e-15) { if(o[a]<n.lo.x[a] || o[a]>n.hi.x[a]) return false; }
  else {
   double t0=(n.lo.x[a]-o[a])/d[a],t1=(n.hi.x[a]-o[a])/d[a];
   if(t0>t1) std::swap(t0,t1);
   near=std::max(near,t0);far=std::min(far,t1);
   if(near>far) return false;
  }
 }
 return true;
}
extern "C" void* surfel_create(const double* p,const double* n,const double* radii,const int32_t* kinds,int count) {
 try {
  std::unique_ptr<BVH> b(new BVH());b->surfels.resize(count);b->ids.resize(count);
  std::iota(b->ids.begin(),b->ids.end(),0);
  for(int i=0;i<count;++i) for(int a=0;a<3;++a) {
   b->surfels[i].p.x[a]=p[3*i+a];b->surfels[i].n.x[a]=n[3*i+a];
  }
  for(int i=0;i<count;++i){b->surfels[i].radius=radii[i];b->surfels[i].kind=kinds[i];}
  if(count) b->build(0,count);
  return b.release();
 } catch(...) {return nullptr;}
}
extern "C" void surfel_destroy(void* handle) {delete static_cast<BVH*>(handle);}
extern "C" void surfel_cast(void* handle,const double* origins,const double* directions,int count,
 double blind,double range,int threads,double* output,uint64_t* stats) {
 auto& b=*static_cast<BVH*>(handle);
 uint64_t boxes=0,disks=0;
 #pragma omp parallel for schedule(static) num_threads(threads) reduction(+:boxes,disks)
 for(int r=0;r<count;++r) {
  const double* o=origins+3*r;const double* d=directions+3*r;
  double best=range;bool hit=false;
  struct Entry{int index;double near;};std::array<Entry,128> stack;int size=0;
  double near=0;++boxes;
  if(!b.nodes.empty() && box(b.nodes[0],o,d,0.,best,near)) stack[size++]={0,near};
  while(size) {
   Entry entry=stack[--size];if(entry.near>best) continue;
   const Node& node=b.nodes[entry.index];
   if(node.left>=0) {
    double t0,t1;boxes+=2;
    bool l=box(b.nodes[node.left],o,d,0.,best,t0),rr=box(b.nodes[node.right],o,d,0.,best,t1);
    // Balanced median tree: max stack depth is far below 128 for any int-sized point map.
    if(l && rr) {
     if(t0<t1) {stack[size++]={node.right,t1};stack[size++]={node.left,t0};}
     else {stack[size++]={node.left,t0};stack[size++]={node.right,t1};}
    } else if(l) stack[size++]={node.left,t0};else if(rr) stack[size++]={node.right,t1};
   } else {
    for(int i=node.begin;i<node.end;++i) {
     ++disks;const Surfel& s=b.surfels[b.ids[i]];
     if(s.kind==1) {
      double projection=0,distance2=0;
      for(int a=0;a<3;++a){double offset=o[a]-s.p.x[a];projection+=offset*d[a];distance2+=offset*offset;}
      double discriminant=projection*projection-distance2+s.radius*s.radius;
      if(discriminant<0)continue;
      double t=-projection-std::sqrt(discriminant);
      if(t<0)t=-projection+std::sqrt(discriminant);
      if(t>=0 && t<=best){best=t;hit=true;}
      continue;
     }
     double denominator=0,numerator=0;
     for(int a=0;a<3;++a) {denominator+=d[a]*s.n.x[a];numerator+=(s.p.x[a]-o[a])*s.n.x[a];}
     if(std::abs(denominator)<1e-12) continue;
     double t=numerator/denominator;
     if(t<0. || t>best) continue;
     double distance2=0;
     for(int a=0;a<3;++a) {double delta=o[a]+t*d[a]-s.p.x[a];distance2+=delta*delta;}
     if(distance2<=s.radius*s.radius*(1+1e-12)) {best=t;hit=true;}
    }
   }
  }
  // An opaque hit inside the output blind range still blocks farther surfaces.
  output[r]=(hit && best>=blind)?best:0;
 }
 stats[0]=boxes;stats[1]=disks;
}
