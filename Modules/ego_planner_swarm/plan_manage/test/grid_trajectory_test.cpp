#include <iostream>
#include <cmath>
#include <stdexcept>
#include <plan_manage/planner_manager.h>

void require(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
int main(int argc, char **argv) {
  ros::init(argc, argv, "grid_trajectory_regression", ros::init_options::NoSigintHandler);
  ros::Time::init();
  try {
    ego_planner::EGOPlannerManager manager;
    manager.pp_.max_vel_ = .6;
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    const Eigen::Vector3d start(.05473511,.05274566,1.02448359), velocity(.0471695,.02907246,-.06929458);
    std::vector<Eigen::Vector3d> grid;
    for (int row=0; row<3; ++row)
      for(int col=0; col<5; ++col) grid.emplace_back((row%2?4-col:col)*8.,row*8.,2.);
    require(manager.planGlobalTrajWaypoints(start,velocity,zero,grid,zero,zero,true), "grid generation failed");
    auto trajectory=manager.global_data_.global_traj_;
    const double join=2.*(grid.front()-start).norm()/.6;
    for(double t=join;t<trajectory.getTimeSum();t+=.02)
      require(std::abs(trajectory.evaluate(t).z()-2.)<1e-7,"level grid reference still overshoots its altitude");
    require((trajectory.evaluate(join)-grid.front()).norm()<1e-7,"join position changed");
    require(trajectory.evaluateVel(join).norm()<1e-7,"join velocity is not zero");
    require(trajectory.evaluateAcc(join).norm()<1e-7,"join acceleration is not zero");
    for(double dt:{-1e-3,1e-3}) {
      require((trajectory.evaluate(join+dt)-grid.front()).norm()<1e-5,"join position discontinuity");
      require(trajectory.evaluateVel(join+dt).norm()<1e-4,"join velocity discontinuity");
      require(trajectory.evaluateAcc(join+dt).norm()<.01,"join acceleration discontinuity");
    }
    for(double t=0;t<join;t+=.01)
      require(trajectory.evaluate(t).allFinite() && trajectory.evaluate(t).z()<=2.+1e-7,"approach altitude overshoots the first grid point");
    require((trajectory.evaluate(0)-start).norm()<1e-8,"initial position changed");
    require((trajectory.evaluateVel(0)-velocity).norm()<1e-8,"initial velocity changed");
    const std::vector<Eigen::Vector3d> short_grid{grid[0],grid[1]};
    require(manager.planGlobalTrajWaypoints(start,velocity,zero,short_grid,zero,zero,true),"two-point cruise failed");
    auto short_route=manager.global_data_.global_traj_;
    for(double t=join;t<short_route.getTimeSum();t+=.1)
      require(std::abs(short_route.evaluate(t).z()-2.)<1e-7,"two-point cruise altitude changed");
    // Translation must preserve shared-world placement for any UAV namespace.
    const Eigen::Vector3d offset(100.,5.,3.);
    for(auto &p:grid) p+=offset;
    require(manager.planGlobalTrajWaypoints(start+offset,velocity,zero,grid,zero,zero,true), "translated grid failed");
    auto translated=manager.global_data_.global_traj_;
    for(double t=0;t<trajectory.getTimeSum();t+=.1)
      require((translated.evaluate(t)-trajectory.evaluate(t)-offset).norm()<1e-7,"world placement was applied incorrectly");
    require(manager.planGlobalTrajWaypoints(grid.front(),zero,zero,grid,zero,zero,true), "already-at-first-point grid failed");
    auto coincident=manager.global_data_.global_traj_;
    for(double t=0;t<coincident.getTimeSum();t+=.1)
      require(std::abs(coincident.evaluate(t).z()-5.)<1e-7,"coincident initial point created an invalid segment");
    // Multi-height paths and ordinary callers must retain their previous coefficients.
    grid[2].z()+=1.;
    require(manager.planGlobalTrajWaypoints(start+offset,velocity,zero,grid,zero,zero),"legacy path failed");
    auto legacy=manager.global_data_.global_traj_;
    require(manager.planGlobalTrajWaypoints(start+offset,velocity,zero,grid,zero,zero,true),"multi-height fallback failed");
    auto fallback=manager.global_data_.global_traj_;
    for(double t=0;t<legacy.getTimeSum();t+=.1)
      require((legacy.evaluate(t)-fallback.evaluate(t)).norm()<1e-9,"multi-height route behavior changed");
    std::cout << "Grid altitude, C2 join, translation and fallback regressions passed" << std::endl;
  } catch(const std::exception &e) { std::cerr<<e.what()<<std::endl; return 1; }
}
