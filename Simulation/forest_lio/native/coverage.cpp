// Sparse ray-trace evidence. Traced voxels are NOT certified free-space volumes.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
struct Key {int32_t x,y,z;bool operator==(const Key& q)const{return x==q.x && y==q.y && z==q.z;}};
struct Hash {size_t operator()(const Key& k)const{
 uint64_t v=uint32_t(k.x)*uint64_t(73856093)^uint32_t(k.y)*uint64_t(19349663)^uint32_t(k.z)*uint64_t(83492791);
 return v^(v>>32);
}};
struct Coverage{double voxel;std::unordered_map<Key,uint8_t,Hash> cells;};
static Key cell(const double* p,double v){return {int32_t(std::floor(p[0]/v)),int32_t(std::floor(p[1]/v)),int32_t(std::floor(p[2]/v))};}
extern "C" void* coverage_create(double voxel){return new Coverage{voxel,{}};}
extern "C" void coverage_destroy(void* handle){delete static_cast<Coverage*>(handle);}
extern "C" void coverage_update(void* handle,const double* origin,const double* endpoints,int count,double blind,double range){
 auto& c=*static_cast<Coverage*>(handle);c.cells[cell(origin,c.voxel)]|=4;
 for(int i=0;i<count;++i){
  const double* endpoint=endpoints+3*i;double direction[3],length=0;
  for(int a=0;a<3;++a){direction[a]=endpoint[a]-origin[a];length+=direction[a]*direction[a];}
  length=std::sqrt(length);if(length<blind || length<1e-12)continue;
  double start[3],end[3];
  for(int a=0;a<3;++a){direction[a]/=length;start[a]=origin[a]+blind*direction[a];end[a]=origin[a]+std::min(length,range)*direction[a];}
  if(length<=range)c.cells[cell(endpoint,c.voxel)]|=2; // Never invent a surface at a clipped long ray.
  Key k=cell(start,c.voxel),finish=cell(end,c.voxel);int coordinate[3]={k.x,k.y,k.z};
  int step[3];double next[3],delta[3];
  for(int a=0;a<3;++a){
   double displacement=end[a]-start[a];step[a]=(displacement>0)-(displacement<0);
   if(!step[a]){next[a]=delta[a]=std::numeric_limits<double>::infinity();}
   else {double edge=(coordinate[a]+(step[a]>0))*c.voxel;next[a]=(edge-start[a])/displacement;delta[a]=c.voxel/std::abs(displacement);}
  }
  int limit=int(std::ceil(3*range/c.voxel))+8;
  for(int iter=0;iter<limit;++iter){
   c.cells[Key{coordinate[0],coordinate[1],coordinate[2]}]|=1;
   if(coordinate[0]==finish.x && coordinate[1]==finish.y && coordinate[2]==finish.z)break;
   double t=std::min(next[0],std::min(next[1],next[2]));if(t>1+1e-12)break;
   for(int a=0;a<3;++a)if(next[a]<=t+1e-12){coordinate[a]+=step[a];next[a]+=delta[a];}
  }
 }
}
extern "C" uint64_t coverage_size(void* handle){return static_cast<Coverage*>(handle)->cells.size();}
extern "C" void coverage_export(void* handle,int32_t* indices,uint8_t* flags){
 int i=0;for(const auto& item:static_cast<Coverage*>(handle)->cells){indices[3*i]=item.first.x;indices[3*i+1]=item.first.y;indices[3*i+2]=item.first.z;flags[i]=item.second;++i;}
}
