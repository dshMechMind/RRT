#ifndef RRT_PLUGIN__RRT_PLUGIN_HPP_
#define RRT_PLUGIN__RRT_PLUGIN_HPP_

#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "visual_tool.hpp"
#include "rrt.hpp"
#include "utility.hpp"
#include "visibility_control.hpp"

namespace rrt_interface
{
MOVEIT_CLASS_FORWARD(RRTPlanningContext);

class RRTPlanningContext : public planning_interface::PlanningContext
{
public:
    RRTPlanningContext(const std::string& name, const std::string& group,
                                           moveit::core::RobotModelConstPtr robotModel,
                                           const rclcpp::Node::SharedPtr& node) :
                                           planning_interface::PlanningContext(name, group),
                                           robotModel_(robotModel),
                                           visualTool_(std::make_shared<planner::VisualTool>(node, robotModel, group)) {}
    ~RRTPlanningContext() override {}

    bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
    bool solve(planning_interface::MotionPlanResponse& res) override;
    bool terminate() override;
    void clear() override;
private:
    void printModelInfo();
    planner::Bounds generateRRTBounds(const moveit::core::JointBoundsVector& jointBoundsVector);
    robot_trajectory::RobotTrajectory path2Trajectry(const std::vector<planner::State>& path);
    std::vector<double> reqGoal2Values(const moveit_msgs::msg::Constraints& constraint);

    moveit::core::RobotModelConstPtr robotModel_;
    std::shared_ptr<planner::VisualTool> visualTool_;
};
}  // namespace rrt_interface

#endif  // RRT_INTERFACE__RRT_NTERFACE_HPP_
