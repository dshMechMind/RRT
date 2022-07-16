#include <chrono>
#include <assert.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "rrt_planning_context.hpp"
#include "log_utils.hpp"
#include "collision_checker.hpp"
#include "fk_panda.hpp"

namespace rrt_interface
{
bool RRTPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{ 
    planning_interface::MotionPlanDetailedResponse detailRes;
    if (solve(detailRes))
    {
        std::cout << "detail_res.trajectory_ size " << detailRes.trajectory_.size() << std::endl;
        res.trajectory_ = detailRes.trajectory_[0]; // TODO: generate multi-segment trajectories
        std::cout << "res.trajectory size: " << res.trajectory_->getWayPointCount() << std::endl;
    }
    res.planning_time_ = detailRes.processing_time_[0];
    res.error_code_ = detailRes.error_code_;
    return true;
}

bool RRTPlanningContext::terminate() { return false; }

void RRTPlanningContext::clear() {}

robot_trajectory::RobotTrajectory RRTPlanningContext::path2Trajectry(const std::vector<planner::State>& path)
{
    auto trajectory = robot_trajectory::RobotTrajectory(robotModel_, getGroupName());
    double dt = 0.01;
    for (const auto& point : path)
    {
        moveit::core::RobotState state(robotModel_);
        state.setJointGroupPositions(group_, point);
        trajectory.addSuffixWayPoint(state, dt);
    }
    std::cout << "rrt trajectory size: " << trajectory.getWayPointCount() << std::endl;
    return trajectory;
}

bool RRTPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
    // printModelInfo();
    auto startTime = std::chrono::steady_clock::now();
    
    const moveit::core::JointModelGroup* jmg = robotModel_->getJointModelGroup(group_);
    planner::RRT rrt(jmg->getActiveVariableCount(), visualTool_);
    LOGD("rrt.dimension %ld", rrt.dimension());
    
    std::vector<const std::vector<moveit::core::VariableBounds>*> jointBoundsVector = jmg->getActiveJointModelsBounds();    
    planner::Bounds bounds{generateRRTBounds(jointBoundsVector)};

    planner::CollisionCheckerPtr collisionChecker(std::make_shared<planner::CollisionChecker>(robotModel_, group_, visualTool_));
    collisionChecker->init(planning_scene_->getWorld());

    const moveit::core::RobotState currentState = planning_scene_->getCurrentState();
    std::vector<double> start = planner::moveit2Planner(currentState, jmg);

    rrt.init(bounds, collisionChecker, start);
    res.trajectory_.clear();
    bool solved = false;

    for (const auto& goalConstraint : request_.goal_constraints)
    {
        std::vector<double> goal = reqGoal2Values(goalConstraint);
        rrt.setGoal(goal);
        solved = rrt.buildRRT();
        std::vector<planner::State> path = rrt.path();
        robot_trajectory::RobotTrajectory trajectory = path2Trajectry(path);
        res.trajectory_.push_back(std::make_shared<robot_trajectory::RobotTrajectory>(trajectory));

        if (!solved)
            break;
    }
    if (solved)
    {
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        std::cout << "rrt SUCCESS!" << std::endl;
    } else {
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        std::cout << "rrt FAILURE!" << std::endl;
    }
    res.description_.push_back("rrt_planner");
    auto endTime = std::chrono::steady_clock::now();
    res.processing_time_.push_back(std::chrono::duration<double>((endTime - startTime)).count()); // s
    std::cout << "processing_time: " << res.processing_time_[0] << " " << (endTime - startTime).count() << std::endl;

    return solved;
}

std::vector<double> RRTPlanningContext::reqGoal2Values(const moveit_msgs::msg::Constraints& constraint)
{
    std::vector<double> state;
    for (const auto& jointConstraint : constraint.joint_constraints)
    {
        state.push_back(jointConstraint.position);
    }
    return state;
}

planner::Bounds RRTPlanningContext::generateRRTBounds(const moveit::core::JointBoundsVector& jointBoundsVector)
{
    planner::Bounds bounds;
    for (const auto& jointBounds : jointBoundsVector)
    {
        assert(jointBounds->size() == 1);
        moveit::core::VariableBounds moveitBound = jointBounds->data()[0];
        bounds.push_back(planner::Bound(moveitBound.min_position_, moveitBound.max_position_));
    }
    return bounds;
}

void RRTPlanningContext::printModelInfo()
{
    const moveit::core::JointModelGroup* jmg = robotModel_->getJointModelGroup(group_);
    for (const auto& name : jmg->getJointModelNames())
    {
        std::cout << "joint: " << name << std::endl;
        const moveit::core::JointModel* jointModel = jmg->getJointModel(name);
        std::cout << "bound: " << jointModel->getVariableBounds().size() << std::endl;
        for (const auto& bound : jointModel->getVariableBounds())
        {
            std::cout << " max_position_: " << bound.max_position_ << " min_position_: " << bound.min_position_
                      << " max_velocity_: " << bound.max_velocity_ << " min_velocity_: " << bound.min_velocity_
                      << " max_acceleration_: " << bound.max_acceleration_ << " min_acceleration_: " << bound.min_acceleration_ <<std::endl;
        }
        std::cout << "boundMsg: " << std::endl;
        for (const auto& boundMsg : jointModel->getVariableBoundsMsg())
        {
            std::cout << " max_position_: " << boundMsg.max_position << " min_position_: " << boundMsg.min_position
                      << " max_velocity_: " << boundMsg.max_velocity
                      << " max_acceleration_: " << boundMsg.max_acceleration <<std::endl;
        }
    }
    for (const auto& name : jmg->getLinkModelNames())
    {
        std::cout << "link: " << name << std::endl;
    }
    for (const auto& name : robotModel_->getVariableNames())
    {
        std::cout << "Variable name: " << name << std::endl;
    }
    std::cout << "getVariableCount: " << jmg->getVariableCount() << std::endl;
    std::cout << "getActiveVariableCount: " << jmg->getActiveVariableCount() << std::endl;
}
}