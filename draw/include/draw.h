//
// Created by sun on 17-6-8.
//

#ifndef DRAW_DRAW_H
#define DRAW_DRAW_H


#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>
// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <eigen_conversions/eigen_msg.h>
#include <descartes_planner/dense_planner.h>
#include <moveit/move_group_interface/move_group.h>
namespace draw {
    typedef std::vector <descartes_core::TrajectoryPtPtr> TrajectoryVec;
    typedef TrajectoryVec::const_iterator TrajectoryIter;


    const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualize_trajectory_curve";
    const std::string TRAJECTORY_ACTION = "joint_trajectory_action";
    const std::string ROBOT_DESCRIPTION = "robot_description";
    const std::string GROUP_NAME        = "manipulator";
    const std::string WORLD_FRAME       = "base_link";
    const std::string TCP_FRAME         = "tool";

    class DemoDraw {
    public:
        // 构造函数
        DemoDraw();

        virtual ~DemoDraw();

        void initRos();

        void initDescartes();

        void generateTrajectory(TrajectoryVec& traj);

        void planPath(TrajectoryVec& input_traj,TrajectoryVec&  output_traj);

        void runPath(const TrajectoryVec& traj);

        void moveHome(descartes_core::TrajectoryPtPtr &start);

    protected:
        static bool createCircle(int num_points, EigenSTL::vector_Affine3d &poses,EigenSTL::vector_Affine3d &vposes);

        trajectory_msgs::JointTrajectory fromDescartesToJointTrajectory(const TrajectoryVec &trajectory,
                                            double time_delay);

        void publishPosesMarkers(const EigenSTL::vector_Affine3d &poses);

    protected:
        std::vector<std::string> names;
        ros::NodeHandle nh_;
        ros::Publisher marker_publisher_;
        descartes_core::RobotModelPtr robot_model_ptr_;
        descartes_planner::DensePlanner planner_;

    };
}


#endif //DRAW_DRAW_H
