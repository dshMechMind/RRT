#pragma once

#include <assert.h>
#include <random>
#include <iostream>
#include <memory>
#include <map>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.h>
#include <fcl/math/transform.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/motion_plan_request.h>

namespace planner{
struct Bound
{
    Bound(const double l, const double h) : low(l), high(h) 
    {
        assert(low < high);
    }
    double low;
    double high;
};

using State = std::vector<double>;

class Bounds
{
public:
    Bounds(const std::vector<Bound>& bounds) : bounds_(bounds) {}
    Bounds() {}

    void push_back(const Bound& bound) { bounds_.push_back(bound); }
    const Bound operator[](std::size_t i) const { return bounds_[i]; }
    std::size_t size() const { return bounds_.size(); }
    void print() const 
    {
        for (const auto& bound : bounds_)
        {
            std::cout << "bound.low " << bound.low << "bound.high " << bound.high << std::endl;
        }
    }
private:
    std::vector<Bound> bounds_;
};

class RNG
{
public:
    /* Generate a random real within given bounds: [\e lower_bound, \e upper_bound) */
    double uniformReal(double lowerBound, double upperBound)
    {
        assert(lowerBound <= upperBound);
        return (upperBound - lowerBound) * uniDist_(generator_) + lowerBound;
    }
    double uniformReal01() { return uniDist_(generator_); }

private:
    std::mt19937 generator_;
    std::uniform_real_distribution<> uniDist_{0.0, std::nextafter(1.0, __DBL_MAX__)}; // [0, 1]
};
class StateSampler
{
public:
    State sampleUniform();
    void setBounds(const Bounds& bounds) { bounds_ = bounds; }

private:
    Bounds bounds_;
    RNG rng_;
};

class Vertex : public std::enable_shared_from_this<Vertex>
{
public:
    Vertex(const State& state, const std::shared_ptr<Vertex>& predecessor = nullptr)
        : state_(state), predecessor_(predecessor)
    {
    }
    Vertex() {};

    Vertex(const Vertex&) = delete;
    Vertex& operator=(const Vertex&) = delete;
    std::shared_ptr<Vertex> getPtr() { return shared_from_this(); }
    void addSuccessor(const std::shared_ptr<Vertex>& s) { successor_.push_back(s); }
    const State& state() const { return state_; }
    const std::shared_ptr<Vertex>& predecessor() const { return predecessor_; }

private:
    
    State state_;
    std::shared_ptr<Vertex> predecessor_;
    std::vector<std::shared_ptr<Vertex>> successor_;
};
struct Edge
{
    Edge(const std::shared_ptr<Vertex>& u, const std::shared_ptr<Vertex>& v) : out(u), in(v) {}
    std::shared_ptr<Vertex> out;
    std::shared_ptr<Vertex> in;
    double cost = 1;
};
class Graph
{
public:
    void clear()
    {
        vertexes_.clear();
        edges_.clear();
    }
    
    void addVertex(const std::shared_ptr<Vertex>& v) { vertexes_.push_back(v); }
    void addEdge(const std::shared_ptr<Vertex>& u, const std::shared_ptr<Vertex>& v) 
    { 
        edges_.push_back(Edge(u, v));
    }
    const std::vector<std::shared_ptr<Vertex>>& vertexes() const { return vertexes_; }
    const std::vector<Edge>& edges() const { return edges_; }
    
private:
    std::vector<std::shared_ptr<Vertex>> vertexes_;
    std::vector<Edge> edges_;
};

class StateValidityChecker
{
public:
    StateValidityChecker() {}
    virtual ~StateValidityChecker() = default;
    virtual bool isValid(const State& state) = 0; 
    virtual void visualize() {};  
};

inline fcl::Transform3f eigen2Fcl(const Eigen::Isometry3d& tf)
{
    fcl::Transform3f fclTf;
    fcl::Vec3f fclTranslation(tf.translation().x(), tf.translation().y(), tf.translation().z());
    Eigen::Quaterniond q(tf.rotation());
    fcl::Quaternion3f fclQuat(q.w(), q.x(), q.y(), q.z());
    fclTf.setTranslation(fclTranslation);
    fclTf.setQuatRotation(fclQuat);
    return fclTf;
}
inline fcl::Transform3f moveit2Fcl(const geometry_msgs::msg::Pose& pose)
{
    fcl::Transform3f fclTf;
    fcl::Vec3f fclTranslation(pose.position.x, pose.position.y, pose.position.z);
    fcl::Quaternion3f fclQuat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    fclTf.setTranslation(fclTranslation);
    fclTf.setQuatRotation(fclQuat);
    return fclTf;
}
inline geometry_msgs::msg::Pose fcl2Moveit(const fcl::Transform3f& fclTf)
{
    geometry_msgs::msg::Pose pose;
    fcl::Vec3f translation = fclTf.getTranslation();
    fcl::Quaternion3f quat = fclTf.getQuatRotation();
    pose.position.x = translation[0];
    pose.position.y = translation[1];
    pose.position.z = translation[2];
    pose.orientation.w = quat[0];
    pose.orientation.x = quat[1];
    pose.orientation.y = quat[2];
    pose.orientation.z = quat[3];
    return pose;
}
inline Eigen::Isometry3d moveit2Eigen(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translate(Eigen::Vector3d{pose.position.x, pose.position.y, pose.position.z});
    Eigen::Quaterniond q{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    tf.rotate(q);
    return tf;
}
inline std::vector<double> moveit2Planner(const moveit::core::RobotState& robotState, const moveit::core::JointModelGroup* jmg)
{
    std::vector<double> state;
    for (const auto& jointModel : jmg->getActiveJointModels())
    {
        const double* valuePtr = robotState.getJointPositions(jointModel);
        double value = valuePtr[0];
        state.push_back(value);
    }
    return state;
}
inline Eigen::Isometry3d rotateZAxis(double angle)
{
    Eigen::AngleAxisd angleAxis(angle, Eigen::Vector3d(0, 0, 1));
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.rotate(angleAxis);
    return tf;
}
}
