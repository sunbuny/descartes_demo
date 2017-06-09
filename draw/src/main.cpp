//
// Created by sun on 17-6-8.
//
#ifdef __i386__
#pragma message("i386 Architecture detected, disabling EIGEN VECTORIZATION")
  #define EIGEN_DONT_VECTORIZE
  #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#else
#pragma message("64bit Architecture detected, enabling EIGEN VECTORIZATION")
#endif
#include "draw.h"
int main(int argc, char** argv){
    ros::init(argc,argv,"draw");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    draw::DemoDraw draw;
    draw.initRos();

    draw.initDescartes();

    while(ros::ok()){
        draw::TrajectoryVec traj;
        draw.generateTrajectory(traj);
        draw::TrajectoryVec output_traj;
        draw.planPath(traj,output_traj);
        draw.moveHome(output_traj[0]);
        draw.runPath(output_traj);
    }
}