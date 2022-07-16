#pragma once

#include <assert.h>
#include <vector>
#include <chrono>
#include <thread>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shapes.h>
#include <fcl/collision.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/octree.h>
#include <Eigen/Core>
#include "utility.hpp"
#include "log_utils.hpp"

namespace planner
{
class FKPanda
{
public:
    FKPanda(moveit::core::RobotModelConstPtr robotModel, const std::string group) : 
    robotModel_(robotModel), group_(group) 
    {
        const moveit::core::JointModelGroup* jmg = robotModel_->getJointModelGroup(group_);
        jmg->printGroupInfo(std::cout);
        activeLinkNames_ = jmg->getLinkModelNames();
        activeLinkNames_.pop_back();

        std::cout << "activeLinkNames" << std::endl;
        for (const auto& name : activeLinkNames_)
            std::cout << name << std::endl;

        linkNames_ = jmg->getLinkModelNames();
        linkNames_.push_back("panda_hand");
        linkNames_.push_back("panda_leftfinger");
        linkNames_.push_back("panda_rightfinger");

        std::cout << "link names" << std::endl;
        for (const auto& name : linkNames_)
            std::cout << name << std::endl;

        linksTfOrigin_.clear();
        for (const auto& name : linkNames_)
        {
            const moveit::core::LinkModel* link = robotModel_->getLinkModel(name);
            Eigen::Isometry3d tf = link->getJointOriginTransform();
            linksTfOrigin_.insert(std::make_pair(name, tf));
        }
    }
    ~FKPanda() {}
    
    void update(const State& state)
    {
        assert(activeLinkNames_.size() == state.size());
        State state_extend = state;
        state_extend.resize(linkNames_.size());
        linksTf_.clear();
        lookupLinktf(state_extend, Eigen::Isometry3d::Identity(), 0);
    }
    const std::map<std::string, Eigen::Isometry3d>& getLinkTf() const { return linksTf_; }
    const std::vector<std::string>& getLinkNames() const { return linkNames_; }
   
private:
    void lookupLinktf(const State& state, const Eigen::Isometry3d& tfPre, std::size_t index)
    {
        std::string name = linkNames_[index];
        assert(name != "");
        Eigen::Isometry3d tf = tfPre * linksTfOrigin_[name] * rotateZAxis(state[index]);
        linksTf_.insert(std::make_pair(name, tf));
        index++;
        if (index < (state.size() - 1))
            lookupLinktf(state, tf, index);
        else if (index == (state.size() - 1))
            lookupLinktf(state, tfPre, index); // panda_leftfinger & panda_rightfinger have the same parent: link panda_hand
    }

    moveit::core::RobotModelConstPtr robotModel_;
    std::string group_;
    std::vector<std::string> linkNames_;
    std::vector<std::string> activeLinkNames_;
    std::map<std::string, Eigen::Isometry3d> linksTf_;
    std::map<std::string, Eigen::Isometry3d> linksTfOrigin_;
};
}
