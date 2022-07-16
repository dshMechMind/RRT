#pragma once

#include <string>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model/joint_model_group.h>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Core>
#include "rclcpp/rclcpp.hpp"
#include "log_utils.hpp"
#include "utility.hpp"
#include "fk_panda.hpp"

namespace planner
{
class VisualTool
{
public:
    VisualTool(const rclcpp::Node::SharedPtr& node, moveit::core::RobotModelConstPtr robotModel, const std::string& group) :
        robotModel_(robotModel), frameId_("panda_link0"), group_(group), visualTool_(node, frameId_, "move_group_tutorial", robotModel), 
        fk_(robotModel, group) {}
        
    void deleteAllMarkers() { visualTool_.deleteAllMarkers(); }
    void visualizeAxis(const State& state, const std::string& poseDesc);
    void visualizeAxis(const geometry_msgs::msg::Pose& pose, const std::string& poseDesc);
    void visualizeAxis(const Eigen::Isometry3d& pose, const std::string& poseDesc);
    void visualizePoses(const std::vector<State>& states);
    void visualizeAxisVector(const std::vector<State>& states);
    void visualizeGraph(const Graph& graph);

private:
    geometry_msgs::msg::Point state2Point(const State& state);

    moveit::core::RobotModelConstPtr robotModel_;
    std::string frameId_;
    std::string group_;
    moveit_visual_tools::MoveItVisualTools visualTool_;
    FKPanda fk_;
};
}
