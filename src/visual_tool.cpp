#include "visual_tool.hpp"

namespace planner
{
void VisualTool::visualizeAxis(const State& state, const std::string& poseDesc)
{
    fk_.update(state);
    std::map<std::string, Eigen::Isometry3d> links_tf = fk_.getLinkTf();
    visualTool_.publishAxisLabeled(links_tf["panda_hand"], poseDesc);
    visualTool_.trigger();
}

void VisualTool::visualizeAxis(const geometry_msgs::msg::Pose& pose, const std::string& poseDesc)
{
    visualTool_.publishAxisLabeled(pose, poseDesc);
    visualTool_.trigger();
}

void VisualTool::visualizeAxis(const Eigen::Isometry3d& pose, const std::string& poseDesc)
{
    visualTool_.publishAxisLabeled(pose, poseDesc);
    visualTool_.trigger();
}

void VisualTool::visualizePoses(const std::vector<State>& states)
{
    std::cout << "path size: " << states.size() << std::endl;
    double scale = 0.002;
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = frameId_;
    sphere.ns = "sphere";
    sphere.id = 2;
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST; // LINE_STRIP
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.color.r = 1.0f;
    sphere.color.g = 0.0f;
    sphere.color.b = 0.0f;
    sphere.color.a = 1.0f;
    for (const auto& state : states)
    {
        sphere.points.push_back(state2Point(state));
    }
    visualTool_.publishMarker(sphere);
    visualTool_.trigger();
}

void VisualTool::visualizeAxisVector(const std::vector<State>& states)
{
    for (const auto& state : states)
    {
        visualizeAxis(state, "");
    }
    visualTool_.trigger();
}

void VisualTool::visualizeGraph(const Graph& graph)
{
    std::cout << "graph vertex size: " << graph.vertexes().size() << std::endl;
    double scale = 0.001;
    visualization_msgs::msg::Marker tree;
    tree.header.frame_id = frameId_;
    tree.ns = "rrt_tree";
    tree.id = 0;
    tree.type = visualization_msgs::msg::Marker::LINE_LIST; // LINE_STRIP
    tree.action = visualization_msgs::msg::Marker::ADD;
    tree.scale.x = scale;
    tree.scale.y = scale;
    tree.scale.z = scale;
    tree.color.r = 1.0f;
    tree.color.g = 1.0f;
    tree.color.b = 0.0f;
    tree.color.a = 1.0f;

    for (const auto& edge : graph.edges())
    {
        State out = edge.out->state();
        tree.points.push_back(state2Point(out));
        State in = edge.in->state();
        tree.points.push_back(state2Point(in));
    }

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = frameId_;
    sphere.ns = "rrt_sphere";
    sphere.id = 1;
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST; // LINE_STRIP
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.color.r = 0.0f;
    sphere.color.g = 1.0f;
    sphere.color.b = 0.0f;
    sphere.color.a = 1.0f;

    for (const auto& vertex : graph.vertexes())
    {
        State state = vertex->state();
        sphere.points.push_back(state2Point(state));
    }

    visualTool_.publishMarker(tree);
    visualTool_.publishMarker(sphere);
    visualTool_.trigger();
}

geometry_msgs::msg::Point VisualTool::state2Point(const State& state)
{
    geometry_msgs::msg::Point point;
    fk_.update(state);
    std::map<std::string, Eigen::Isometry3d> links_tf = fk_.getLinkTf();
    Eigen::Isometry3d tf = links_tf["panda_hand"];
    point.x = tf.translation().x();
    point.y = tf.translation().y();
    point.z = tf.translation().z();
    return point;
}
}
