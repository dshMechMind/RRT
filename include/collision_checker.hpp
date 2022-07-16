#pragma once

#include <assert.h>
#include <chrono>
#include <thread>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <geometric_shapes/shapes.h>
#include <fcl/collision.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <Eigen/Core>
#include "utility.hpp"
#include "log_utils.hpp"
#include "visual_tool.hpp"
#include "fk_panda.hpp"

namespace planner
{
MOVEIT_CLASS_FORWARD(CollisionChecker);
class CollisionChecker : public StateValidityChecker
{
public:
    CollisionChecker(moveit::core::RobotModelConstPtr robotModel, const std::string& group, const std::shared_ptr<VisualTool>& visualTool) : 
    StateValidityChecker(), robotModel_(robotModel), visualTool_(visualTool), fk_(robotModel, group) {}
    ~CollisionChecker() override {}
    
    void init(const collision_detection::WorldConstPtr& world); 
    bool isValid(const State& state) override;
    void visualize() override;
   
private:
    std::shared_ptr<fcl::CollisionGeometry> constructFCLGeometry(const shapes::ShapeConstPtr& shape);
    std::vector<std::shared_ptr<fcl::CollisionGeometry>> constructFCLGeometryRobotLink(const std::string& name);
    void constructFCLGeometryRobot();
    void constructFCLObjectRobotLink(const std::string name, const Eigen::Isometry3d& tf);
    void constructFCLObjectRobot(const State& state);
    void constructFCLObjectWorld(const collision_detection::WorldConstPtr& world);
    bool checkShapeGroupCollision(const std::pair<std::string, std::vector<std::shared_ptr<fcl::CollisionObject>>>& g1,
                                  const std::pair<std::string, std::vector<std::shared_ptr<fcl::CollisionObject>>>& g2);
    bool robotSelfCollisionCheck();
    bool robotWorldCollisionCheck();

    void FKUT(const State& state);
    
    moveit::core::RobotModelConstPtr robotModel_;
    std::shared_ptr<VisualTool> visualTool_;
    FKPanda fk_;
    std::vector<std::string> linkNames_;
    std::map<std::string, Eigen::Isometry3d> linksTf_;
    collision_detection::AllowedCollisionMatrix acm_;
    std::map<std::string, std::vector<std::shared_ptr<fcl::CollisionGeometry>>> fclGeometriesRobot_;
    std::vector<std::shared_ptr<fcl::CollisionGeometry>> fclGeometriesWorld_;
    std::map<std::string, std::vector<std::shared_ptr<fcl::CollisionObject>>> fclObjsRobotMap_;
    std::map<std::string, std::vector<std::shared_ptr<fcl::CollisionObject>>> fclObjsWorldMap_;
}; 
}
