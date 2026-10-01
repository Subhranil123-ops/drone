// Closed-loop kinematic check of perception+planner using ONLY scans/pose/goal.
// The "world" (random cylinders) exists only inside the ray-caster that fakes the LiDAR.
#include "lidar_processor.hpp"
#include "navigator.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>
static const double PI=3.14159265358979323846, INF=std::numeric_limits<double>::infinity();
struct Cyl{double n,e,r;};
static double cast(const std::vector<Cyl>&w,double pn,double pe,double dn,double de,double maxR){
  double best=maxR;
  for(auto&c:w){ double fn=pn-c.n, fe=pe-c.e; double b=fn*dn+fe*de, cc=fn*fn+fe*fe-c.r*c.r;
    double disc=b*b-cc; if(disc<0) continue; double t=-b-std::sqrt(disc); if(t>0.1&&t<best) best=t; }
  return best; }
int main(int argc,char**argv){
  int layouts=argc>1?atoi(argv[1]):200; std::mt19937 rng(7);
  LidarProcessor lp(72,12.0); int ok=0, coll=0, timeout=0; double minClearAll=INF;
  for(int L=0;L<layouts;++L){
    std::uniform_real_distribution<double> un(3,38), ue(-12,12), ur(0.8,2.0), uy(-PI,PI);
    std::vector<Cyl> w; for(int tries=0;tries<400&&w.size()<9;++tries){ Cyl c{un(rng),ue(rng),ur(rng)};
      bool fine=std::hypot(c.n,c.e)>c.r+3 && std::hypot(c.n-40,c.e)>c.r+3;
      for(auto&o:w){ if(std::hypot(c.n-o.n,c.e-o.e)<c.r+o.r+3.5) fine=false; }
      if(fine) w.push_back(c);} 
    double pn=0,pe=0,yaw=uy(rng), vn=0,ve=0; Goal g; g.northM=40; g.eastM=0; g.downM=-3; g.positionToleranceM=1.0;
    Navigator nav(2.2,1.0,1.2); VehicleState s; s.localZ=-3; bool reached=false, hit=false; double minClear=INF; const double dt=0.1;
    for(int step=0;step<3000;++step){
      LidarFrame f; f.horizontalCount=360; f.verticalCount=16; f.rangeMaxM=20;
      for(int v=0;v<16;++v){ double el=-PI/4+v*(PI/2/15);
        for(int h=0;h<360;++h){ double az=-PI+h*(2*PI/360); double bearing=yaw-az; // gz az is CCW(left); NED bearing is CW
          static const std::vector<Cyl> empty; double d=cast(getenv("BLIND")?empty:w,pn,pe,std::cos(bearing),std::sin(bearing),20.0); f.beams.push_back({getenv("MIRROR")?-az:az,el,d>=20.0?INF:d}); } }
      s.localX=pn; s.localY=pe; s.yaw=yaw; auto cmd=nav.update(s,g,lp.analyze(f));
      if(cmd.reached){reached=true;break;}
      double fw=cmd.forwardMps, rt=cmd.rightMps; // BODY_NED -> NED
      double tvn=std::cos(yaw)*fw-std::sin(yaw)*rt, tve=std::sin(yaw)*fw+std::cos(yaw)*rt;
      vn+=(tvn-vn)*0.35; ve+=(tve-ve)*0.35; pn+=vn*dt; pe+=ve*dt;
      for(auto&c:w){ double cl=std::hypot(pn-c.n,pe-c.e)-c.r; minClear=std::min(minClear,cl); if(cl<0.4) hit=true; }
      if(hit){ if(getenv("DBG")){ printf("COLL layout=%d step=%d pos=(%.1f,%.1f) yaw=%.0f vel=(%.2f,%.2f) cmd f=%.2f r=%.2f ncyl=%zu\n",L,step,pn,pe,yaw*180/PI,vn,ve,cmd.forwardMps,cmd.rightMps,w.size()); } break; } }
    minClearAll=std::min(minClearAll,minClear);
    if(hit) ++coll; else if(reached) ++ok; else ++timeout; }
  printf("layouts=%d reached=%d collisions=%d timeouts/stuck=%d minClearance=%.2f m\n",layouts,ok,coll,timeout,minClearAll);
  return coll?1:0; }
