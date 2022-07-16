#include <math.h>
#include <iomanip>
#include "rrt.hpp"
#include "log_utils.hpp"

MOVEIT_CLASS_FORWARD(CollisionChecker);
namespace planner{
namespace {
double sign(double x)
{
    if (x < -__DBL_MIN__)
        return -1.0;
    if (x > __DBL_MIN__)
        return 1.0;
    else return 0;
}

void printState(const State& s, const std::string& desc = "" )
{
    std::cout << desc << std::endl;
    for (const auto& v : s)
    {
        std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << v << ", ";
    }
    std::cout << std::endl;

}

// [-M_PI, M_PI]
double normalizeAngle(double angle)
{
    double a = fmod(angle, 2.0 * M_PI);
    if ((-M_PI <= a) && (a <= M_PI))
        return a;
    if (a < - M_PI)
        return a + 2 * M_PI;
    else
        return a - 2 * M_PI;
}
}

RRT::RRT(const size_t dimension, const std::shared_ptr<VisualTool>& visualTool) :
         dimension_(dimension), visualTool_(visualTool)
{
    LOGD();
    path_.clear();
    steps_ = 3000; // 3000
    stepSize_ = 0.01;
    goalThreshold_ = 0.03;
    goalBias_ = 0.1;
}

void RRT::init(const Bounds& bounds, const std::shared_ptr<StateValidityChecker>& stateValidityChecker, const State& start)
{
    setBounds(bounds);
    setStateValidityChecker(stateValidityChecker);
    setStart(start);
}

void RRT::setStart(const State& start)
{
    assert(start.size() == dimension_);
    start_ = start;
    printState(start_, "start");
}

void RRT::setGoal(const State& goal)
{
    assert(goal.size() == dimension_);
    goal_ = goal;
    printState(goal_, "goal");
}

bool RRT::buildRRT()
{
    LOGD();
    updateVisualizer();

    graph_.clear();
    path_.clear();
    
    if (!stateValidityChecker(start_))
    {
        std::cout << "start state is in collision" << std::endl;
        return false;
    }
    if (!stateValidityChecker(goal_))
    {
        std::cout << "goal state is in collision" << std::endl;
        return false;
    }
    auto startVertex = std::make_shared<Vertex>(start_);
    graph_.addVertex(startVertex);
    LOGD("graph size: %ld", graph_.vertexes().size());
    unsigned i = 0; 
    Status status = Status::Trapped;
    LOGD("*************************************");
    while ((i < steps_) && (status != Status::Reached)) 
    {
        State xRandom;
        if (rng_.uniformReal01() < goalBias_)
             xRandom = goal_;
        else
            xRandom = randomState();
    
        LOGD("******* Step%d*******", i);
        LOGD(" xRandom: ");
        // printState(xRandom);
        status = extend(xRandom, graph_);
        LOGD(" status: %d", status);
        switch (status)
        {
            case Status::Reached:
            {
                updateVisualizer();
                return true;
            }
            case Status::Advanced:
            {
                break;
            }
            case Status::Trapped:
            {
                break;
            }
        }
        i++;
    }
    
    return false;
}

RRT::Status RRT::extend(const State& xRandom, Graph& graph)
{
    LOGD();
    const std::shared_ptr<Vertex> vNear = nearestVertex(graph, xRandom);
    LOGD("vNear:");
    // printState(vNear->state());
    State xNew = newState(xRandom, vNear->state(), stepSize_);
    LOGD("xNew:");
    // printState(xNew);
    // visualTool_->visualizeAxis(xNew, "xNew");
    bool isValid = stateValidityChecker(xNew);
    LOGD("isValid: %d", isValid);
    if (isValid) 
    {
        auto vNew = std::make_shared<Vertex>(xNew, vNear);
        vNear->addSuccessor(vNew);
        Edge edgeNew(vNear, vNew);
        graph.addVertex(vNew);
        graph.addEdge(vNear, vNew);
        path_.clear();
        updatePath(graph_.vertexes().back());
        // updateVisualizer();

        if (goalSatisfied(xNew))
            return Reached;
        return Advanced;
    }
    return Trapped;
}

void RRT::updatePath(const std::shared_ptr<Vertex>& v)
{
    path_.push_back(v->state());
    const std::shared_ptr<Vertex> nextVertex = v->predecessor();
    if (nextVertex)
        updatePath(nextVertex);
    else {
        std::reverse(std::begin(path_), std::end(path_));
        LOGD("rrt path size: %ld", path_.size());
        return;  
    }
}

void RRT::updateVisualizer()
{
    visualTool_->deleteAllMarkers();
    visualTool_->visualizeAxis(start_, "start");
    visualTool_->visualizeAxis(goal_, "goal");
    visualTool_->visualizeGraph(graph_);
    visualTool_->visualizePoses(path_);
    // stateValidityChecker_->visualize();
}

const std::shared_ptr<Vertex>& RRT::nearestVertex(const Graph& graph, const State& xRandom)
{
    assert(graph.vertexes().size() >= 1);
    double cost = __DBL_MAX__;
    unsigned minIndex = 0;
    for (size_t i = 0; i < graph.vertexes().size(); i++) 
    {
        double dis = distanceL2(graph.vertexes()[i]->state(), xRandom);
        if (cost > dis) 
        {
            cost = dis;
            minIndex = i;
        }
    }
    LOGD("nearestVertex cost: %f, index: %d", cost, minIndex);
    return graph.vertexes()[minIndex];
}

bool RRT::stateValidityChecker(const State& state)
{
    return stateValidityChecker_->isValid(state);
}

bool RRT::goalSatisfied(const State& x)
{
    assert(x.size() == dimension_);
    for (size_t i = 0; i < dimension_; i++)
    {
        if (abs(normalizeAngle(x[i] - goal_[i])) > goalThreshold_)
            return false;
    }
    return true;
}

double RRT::distanceL1(const State& s1, const State& s2)
{
    // L1 norm
    assert(s1.size() == dimension_);
    assert(s2.size() == dimension_);
    double distance = 0.0;
    for (size_t i = 0; i < dimension_; i++)
    {
        distance += abs(normalizeAngle(s1[i] - s2[i]));
    }
    return distance;
}

double RRT::distanceL2(const State& s1, const State& s2)
{
    // L2 norm
    assert(s1.size() == dimension_);
    assert(s2.size() == dimension_);
    double distance = 0.0;
    Eigen::VectorXd residual(dimension_);
    for (size_t i = 0; i < dimension_; i++)
    {
        residual[i] = normalizeAngle(s1[i] - s2[i]);
    }
    distance = residual.norm();
    return distance;
}

double RRT::distanceLinf(const State& s1, const State& s2)
{
    // Linf norm
    assert(s1.size() == dimension_);
    assert(s2.size() == dimension_);
    double distance = 0.0;
    for (size_t i = 0; i < dimension_; i++)
    {
        double residuum = abs(normalizeAngle(s1[i] - s2[i]));
        if (distance < residuum)
            distance = residuum;
    }
    return distance;
}

State RRT::randomState() { return stateSampler_.sampleUniform(); }

State RRT::newState(const State& xRandom, const State& xNear, const double stepSize)
{
    assert(xRandom.size() == dimension_);
    assert(xNear.size() == dimension_);
    State xNew;
    for (size_t i = 0; i < dimension_; i++)
    {
        double delta = stepSize * sign(normalizeAngle(xRandom[i] - xNear[i])) + xNear[i];
        xNew.push_back(delta);
    }
    return xNew;
}

void RRT::printPath() const
{
    for (size_t i = 0; i < path_.size(); i++)
    {
        LOGD("path[%ld]", i);
        printState(path_[i]);
    }
}

State StateSampler::sampleUniform()
{
    LOGD();
    State state;
    for (size_t i = 0; i < bounds_.size(); i++)
    {
        double d = rng_.uniformReal(bounds_[i].low, bounds_[i].high);
        state.push_back(d);
    }
    return state;
}
}
