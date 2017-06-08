// Core ros functionality like ros::init and spin
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

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

/**
 * Generates an completely defined (zero-tolerance) cartesian point from a pose
 */
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d &pose);

/**
 * Generates a cartesian point with free rotation about the Z axis of the EFF frame
 */
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d &pose);

/**
 * Translates a descartes trajectory to a ROS joint trajectory
 */
trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec &trajectory, const descartes_core::RobotModel &model,
                     const std::vector<std::string> &joint_names, double time_delay);

void publishPosesMarkers(const EigenSTL::vector_Affine3d &poses, ros::Publisher &marker_publisher_);

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory &trajectory);

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "descartes_tutorial");
    ros::NodeHandle nh;
    ros::Publisher marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualize_trajectory_curve", 1,
                                                                                    true);
    // Required for communication with moveit components
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 1. Define sequence of points

    TrajectoryVec points;
    EigenSTL::vector_Affine3d poses;

    for(unsigned int i = 0; i < 50; ++i){
        Eigen::Affine3d pose,tmp;
        double angle = 0;
        angle = 2 * 3.1415 / 50 * i;
        pose = Eigen::Translation3d(0.1 * cos(angle), 0.1 * sin(angle), 1.2125 );
        tmp = pose * Eigen::Translation3d(0,0, 0.11);
        poses.push_back(tmp);
        descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
        points.push_back(pt);
    }
    publishPosesMarkers(poses, marker_publisher);

    // 2. Create a robot model and initialize it
    descartes_core::RobotModelPtr model(new descartes_moveit::MoveitStateAdapter);

    // Name of description on parameter server. Typically just "robot_description".
    const std::string robot_description = "robot_description";

    // name of the kinematic group you defined when running MoveitSetupAssistant
    const std::string group_name = "manipulator";

    // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
    const std::string world_frame = "base_link";

    // tool center point frame (name of link associated with tool)
    const std::string tcp_frame = "tool";

    if (!model->initialize(robot_description, group_name, world_frame, tcp_frame)) {
        ROS_INFO("Could not initialize robot model");
        return -1;
    }

    // 3. Create a planner and initialize it with our robot model
    descartes_planner::DensePlanner planner;
    planner.initialize(model);

    // 4. Feed the trajectory to the planner
    if (!planner.planPath(points)) {
        ROS_ERROR("Could not solve for a valid path");
        return -2;
    }

    TrajectoryVec result;
    if (!planner.getPath(result)) {
        ROS_ERROR("Could not retrieve path");
        return -3;
    }

    // 5. Translate the result into a type that ROS understands
    // Get Joint Names
    std::vector<std::string> names;
    nh.getParam("controller_joint_names", names);
    // Generate a ROS joint trajectory with the result path, robot model, given joint names,
    // a certain time delta between each trajectory point
    trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 1.0);

    // 6. Send the ROS trajectory to the robot for execution
    if (!executeTrajectory(joint_solution)) {
        ROS_ERROR("Could not execute trajectory!");
        return -4;
    }

    // Wait till user kills the process (Control-C)

    ROS_INFO("Done!");
    return 0;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d &pose) {
    using namespace descartes_core;
    using namespace descartes_trajectory;

    return TrajectoryPtPtr(new CartTrajectoryPt(TolerancedFrame(pose)));
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d &pose) {
    using namespace descartes_core;
    using namespace descartes_trajectory;
    return TrajectoryPtPtr(new AxialSymmetricPt(pose, M_PI / 2.0 - 0.0001, AxialSymmetricPt::Z_AXIS));
}

trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec &trajectory,
                     const descartes_core::RobotModel &model,
                     const std::vector<std::string> &joint_names,
                     double time_delay) {
    // Fill out information about our trajectory
    trajectory_msgs::JointTrajectory result;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = "world_frame";
    result.joint_names = joint_names;

    // For keeping track of time-so-far in the trajectory
    double time_offset = 0.0;
    // Loop through the trajectory
    for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it) {
        // Find nominal joint solution at this point
        std::vector<double> joints;
        it->get()->getNominalJointPose(std::vector<double>(), model, joints);

        // Fill out a ROS trajectory point
        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = joints;
        // velocity, acceleration, and effort are given dummy values
        // we'll let the controller figure them out
        pt.velocities.resize(joints.size(), 0.0);
        pt.accelerations.resize(joints.size(), 0.0);
        pt.effort.resize(joints.size(), 0.0);
        // set the time into the trajectory
        pt.time_from_start = ros::Duration(time_offset);
        // increment time
        time_offset += time_delay;

        result.points.push_back(pt);
    }

    return result;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory &trajectory) {
    // Create a Follow Joint Trajectory Action Client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
    if (!ac.waitForServer(ros::Duration(2.0))) {
        ROS_ERROR("Could not connect to action server");
        return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance = ros::Duration(1.0);

    ac.sendGoal(goal);

    if (ac.waitForResult(
            goal.trajectory.points[goal.trajectory.points.size() - 1].time_from_start + ros::Duration(5))) {
        ROS_INFO("Action server reported successful execution");
        return true;
    } else {
        ROS_WARN("Action server could not execute trajectory");
        return false;
    }
}


void publishPosesMarkers(const EigenSTL::vector_Affine3d &poses, ros::Publisher &marker_publisher_) {
    // creating rviz markers
    visualization_msgs::Marker z_axes, y_axes, x_axes, line;
    visualization_msgs::MarkerArray markers_msg;
    // 绘制离散的线段
    z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
    // namespace 的确定
    z_axes.ns = y_axes.ns = x_axes.ns = "axes";
    z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
    z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
    // frame_id是设置的标记点关于参考坐标系的名字
    z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = "base_link";
    // 只需要设定scale.x参数，用来定义线宽
    z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = 0.001;

    // z properties
    z_axes.id = 0;
    z_axes.color.r = 0;
    z_axes.color.g = 0;
    z_axes.color.b = 1;
    z_axes.color.a = 1;

    // y properties
    y_axes.id = 1;
    y_axes.color.r = 0;
    y_axes.color.g = 1;
    y_axes.color.b = 0;
    y_axes.color.a = 1;

    // x properties
    x_axes.id = 2;
    x_axes.color.r = 1;
    x_axes.color.g = 0;
    x_axes.color.b = 0;
    x_axes.color.a = 1;

    // line properties 绘制直线
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.ns = "line";
    line.action = visualization_msgs::Marker::ADD;
    line.lifetime = ros::Duration(0);
    line.header.frame_id = "world";
    line.scale.x = 0.001;
    line.id = 0;
    // 黄色
    line.color.r = 1;
    line.color.g = 1;
    line.color.b = 0;
    line.color.a = 1;

    // creating axes markers
    // 参数points是设定点的状态的参数 在直线中是相邻的两个点
    // 申请两倍齐次矩阵用来存放线的开始和结束点
    z_axes.points.reserve(2 * poses.size());
    y_axes.points.reserve(2 * poses.size());
    x_axes.points.reserve(2 * poses.size());
    line.points.reserve(poses.size());
    // Point类型 x y z 向量
    geometry_msgs::Point p_start, p_end;
    double distance = 0;
    Eigen::Affine3d prev = poses[0];
    for (unsigned int i = 0; i < poses.size(); i++) {
        const Eigen::Affine3d &pose = poses[i];
        distance = (pose.translation() - prev.translation()).norm();

        tf::pointEigenToMsg(pose.translation(), p_start);

        if (distance > 0.02) {
            Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(0.01, 0, 0);
            tf::pointEigenToMsg(moved_along_x.translation(), p_end);
            x_axes.points.push_back(p_start);
            x_axes.points.push_back(p_end);

            Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0, 0.01, 0);
            tf::pointEigenToMsg(moved_along_y.translation(), p_end);
            y_axes.points.push_back(p_start);
            y_axes.points.push_back(p_end);

            Eigen::Affine3d moved_along_z = pose * Eigen::Translation3d(0, 0, 0.01);
            tf::pointEigenToMsg(moved_along_z.translation(), p_end);
            z_axes.points.push_back(p_start);
            z_axes.points.push_back(p_end);

            // saving previous
            prev = pose;
        }

        line.points.push_back(p_start);
    }

    markers_msg.markers.push_back(x_axes);
    markers_msg.markers.push_back(y_axes);
    markers_msg.markers.push_back(z_axes);
    markers_msg.markers.push_back(line);

    marker_publisher_.publish(markers_msg);
}