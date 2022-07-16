#pragma once

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model/joint_model_group.h>

#include "visual_tool.hpp"
#include "utility.hpp"
#include "collision_checker.hpp"

namespace planner{
class RRT
{
public:
    enum Status { Reached, Advanced, Trapped };
    RRT(const size_t dimension, const std::shared_ptr<VisualTool>& visual_tool);
    
    void init(const Bounds& bounds, const std::shared_ptr<StateValidityChecker>& stateValidityChecker, const State& start);
    bool buildRRT();
    void setStart(const State& start);
    void setGoal(const State& goal);
    const std::vector<State>& path() const { return path_; }
    void printPath() const;
    size_t  dimension() const { return dimension_; }
    void setBounds(const Bounds& bounds) { stateSampler_.setBounds(bounds); }
    void setStateValidityChecker(const std::shared_ptr<StateValidityChecker>& stateValidityChecker)
    {
        stateValidityChecker_ = stateValidityChecker;
        if (!stateValidityChecker_)
            std::cout << "setStateValidityChecker failed" << std::endl;
        std::cout << "setStateValidityChecker success" << std::endl;
    }

private:
    const std::shared_ptr<Vertex>& nearestVertex(const Graph& graph, const State& xRandom);
    State newState(const State& xRandom, const State& xNear, const double stepSize);
    State randomState();
    double distanceL1(const State& s1, const State& s2);
    double distanceL2(const State& s1, const State& s2);
    double distanceLinf(const State& s1, const State& s2);
    Status extend(const State& xRandom, Graph& graph);
    bool stateValidityChecker(const State& state);
    bool goalSatisfied(const State& x);
    void updatePath(const std::shared_ptr<Vertex>& v);
    void updateVisualizer();
    
    // The fraction of time the goal is picked as the state to expand towards (if such a state
    // is available)
    double goalBias_{0.05};
    double goalThreshold_{0.05};
    size_t dimension_; 
    State start_;
    State goal_;
    unsigned steps_{1000};
    Graph graph_;
    std::vector<State> path_;
    StateSampler stateSampler_;
    double stepSize_;
    RNG rng_;
    std::shared_ptr<StateValidityChecker> stateValidityChecker_;
    std::shared_ptr<VisualTool> visualTool_;
};
}
