#include "lidar_processor.hpp"
#include "navigator.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
static const double PI=3.14159265358979323846, INF=std::numeric_limits<double>::infinity();
// Build a 360x16 frame. wall(az_left_positive_rad) returns range or INF.
template<class F> LidarFrame frame(F rangeAtAz){
  LidarFrame f; f.horizontalCount=360; f.verticalCount=16; f.rangeMaxM=20;
  for(int v=0;v<16;++v){ double el=-PI/4+v*(PI/2/15);
    for(int h=0;h<360;++h){ double az=-PI+h*(2*PI/360);
      f.beams.push_back({az,el,rangeAtAz(az,el)}); } }
  return f; }
int main(){
  LidarProcessor lp(72,12.0); Navigator nav(2.2,1.0,1.2);
  VehicleState s; s.yaw=0; s.localX=0; s.localY=0; s.localZ=-3;
  Goal g; g.northM=20; g.eastM=0; g.downM=-3; g.positionToleranceM=1;
  // T1: totally open air (all INF), goal straight ahead
  auto a1=lp.analyze(frame([](double,double){return INF;}));
  auto c1=nav.update(s,g,a1);
  printf("T1 open air: fwd=%.2f right=%.2f (expect fwd>0)\n",c1.forwardMps,c1.rightMps);
  // T2: wall on the LEFT only (az +90deg, left-positive per gz), goal ahead; free elsewhere has returns at 15m
  auto left=[](double az,double){ return (az>PI/4&&az<3*PI/4)?1.5:15.0; };
  Navigator n2(2.2,1.0,1.2);
  auto a2=lp.analyze(frame(left)); s.localZ=-3;
  g.northM=0; g.eastM=20; // goal to the RIGHT (east), yaw 0 => body right
  auto c2=n2.update(s,g,a2);
  printf("T2 goal right, wall left: fwd=%.2f right=%.2f (expect right>0)\n",c2.forwardMps,c2.rightMps);
  // T3: wall on the RIGHT only (az -90deg), goal to the right -> must NOT go right
  auto rightWall=[](double az,double){ return (az<-PI/4&&az>-3*PI/4)?1.5:15.0; };
  Navigator n3(2.2,1.0,1.2);
  auto a3=lp.analyze(frame(rightWall));
  auto c3=n3.update(s,g,a3);
  printf("T3 goal right, wall right at 1.5m: fwd=%.2f right=%.2f (expect right<=0 or ~0)\n",c3.forwardMps,c3.rightMps);
  // T4: altitude sign. vehicle at z=0 (ground), goal down=-3 (3 m up): must command negative down vel
  VehicleState s4; s4.localZ=0; Goal g4; g4.northM=20; g4.eastM=0; g4.downM=-3;
  Navigator n4(2.2,1.0,1.2);
  auto c4=n4.update(s4,g4,lp.analyze(frame([](double,double){return 15.0;})));
  printf("T4 need climb 3m: downMps=%.2f (expect <0)\n",c4.downMps);
  
  // T5: unknown obstacle placement sweep: wall of random azimuth/width at 1.8 m; goal ahead.
  // Verify chosen heading never points inside the (inflated) blocked arc.
  int bad=0, runs=0; unsigned seed=12345;
  for(int t=0;t<500;++t){ seed=seed*1103515245u+12345u; double c=((seed>>8)%3600)/3600.0*2*PI-PI;
    seed=seed*1103515245u+12345u; double w=(20+((seed>>8)%80))*PI/180.0; // 20..100 deg wide
    auto fr=frame([&](double az,double){ double d=std::fabs(std::remainder(az-c,2*PI)); return d<w/2?1.8:INF; });
    VehicleState sv; sv.yaw=0; sv.localZ=-3; Goal gg; gg.northM=20; gg.eastM=0; gg.downM=-3;
    Navigator nn(2.2,1.0,1.2); auto cc=nn.update(sv,gg,lp.analyze(fr));
    if(cc.forwardMps==0&&cc.rightMps==0) continue;
    ++runs;
    double hdg_right=std::atan2(cc.rightMps,cc.forwardMps); double hdg_gz=-hdg_right; // back to gz azimuth
    double d=std::fabs(std::remainder(hdg_gz-c,2*PI));
    if(d<w/2) ++bad; }
  printf("T5 random walls: %d/%d commands pointed into a blocked arc (expect 0)\n",bad,runs);
  return 0; }
