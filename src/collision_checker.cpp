#include <moveit/robot_model/joint_model_group.h>
#include "collision_checker.hpp"
#include "log_utils.hpp"

namespace planner
{
void CollisionChecker::init(const collision_detection::WorldConstPtr& world)
{
    linkNames_ = fk_.getLinkNames();
    const std::vector<std::string>& collision_links = robotModel_->getLinkModelNamesWithCollisionGeometry();
    // Use default collision operations in the SRDF to setup the acm
    acm_.setEntry(collision_links, collision_links, false);
    // allow collisions for pairs that have been disabled
    const std::vector<srdf::Model::DisabledCollision>& dc = robotModel_->getSRDF()->getDisabledCollisionPairs();
    for (const srdf::Model::DisabledCollision& it : dc)
        acm_.setEntry(it.link1_, it.link2_, true);
    acm_.setEntry("panda_link8", true);
    acm_.print(std::cout);
    constructFCLGeometryRobot();
    constructFCLObjectWorld(world);
}

void CollisionChecker::FKUT(const State& state)
{
    moveit::core::RobotState robot_state(robotModel_);
    const moveit::core::JointModelGroup* jmg = robotModel_->getJointModelGroup("panda_arm");
    robot_state.setJointGroupPositions(jmg, state);

    for (const auto& name : linkNames_)
    {
        Eigen::Isometry3d pose = robot_state.getGlobalLinkTransform(name);
        std::cout << name << std::endl;
        std::cout << "translation: " << std::endl;
        std::cout << pose.translation() << std::endl;
        std::cout << "rotation: " << std::endl;
        std::cout << pose.rotation().eulerAngles(0, 1, 2) << std::endl;
        visualTool_->visualizeAxis(pose, "");
    }
}

bool CollisionChecker::isValid(const State& state)
{
    constructFCLObjectRobot(state);

    bool valid1 = robotSelfCollisionCheck();
    bool valid2 = robotWorldCollisionCheck();
    
    return (valid1 && valid2);
}

bool CollisionChecker::robotSelfCollisionCheck()
{
    collision_detection::AllowedCollision::Type allowedCollision;

    for (const auto& link1 : fclObjsRobotMap_)
    {
        for (const auto& link2 : fclObjsRobotMap_)
        {
            bool found = acm_.getAllowedCollision(link1.first, link2.first, allowedCollision);
            if (found)
            {
                if (allowedCollision == collision_detection::AllowedCollision::Type::NEVER)
                {
                    bool collide = checkShapeGroupCollision(link1, link2);
                    if (collide)
                        return false;
                }
            }
        }
    }
    return true;
}

bool CollisionChecker::robotWorldCollisionCheck()
{
    for (const auto& link1 : fclObjsWorldMap_)
    {
        for (const auto& link2 : fclObjsRobotMap_)
        {
            bool collide = checkShapeGroupCollision(link1, link2);
            if (collide)
                return false;
        }
    }
    return true;
}

bool CollisionChecker::checkShapeGroupCollision(const std::pair<std::string, std::vector<std::shared_ptr<fcl::CollisionObject>>>& g1,
                                const std::pair<std::string, std::vector<std::shared_ptr<fcl::CollisionObject>>>& g2)
{
    std::vector<std::shared_ptr<fcl::CollisionObject>> g1Shapes = g1.second;
    std::vector<std::shared_ptr<fcl::CollisionObject>> g2Shapes = g2.second;
    for (const auto& g1Shape : g1Shapes)
    {
        for (const auto& g2Shape : g2Shapes)
        {
            fcl::CollisionRequest request;
            request.enable_contact = true;
            fcl::CollisionResult result;
            fcl::CollisionObject* o1 = g1Shape.get();
            fcl::CollisionObject* o2 = g2Shape.get();
            fcl::collide(o1, o2, request, result);
            if (result.isCollision())
            {
                std::cout << "collide! " << g1.first << " & " << g2.first << std::endl;
                // visualTool_->visualizeAxis(fclTf2Pose(g1_shape->getTransform()), g1.first);
                // visualTool_->visualizeAxis(fclTf2Pose(g2_shape->getTransform()), g2.first);
                return true;
            }
        }
    }
    return false;
}

void CollisionChecker::visualize()
{
    std::vector<std::shared_ptr<fcl::CollisionObject>> objs = fclObjsRobotMap_["panda_hand"];
    
    if (objs.size() >= 1)
    {
        std::shared_ptr<fcl::CollisionObject> obj = objs[0];
        fcl::Transform3f fcl_tf = obj->getTransform();
        visualTool_->visualizeAxis(fcl2Moveit(fcl_tf), "R");
    } else {
        std::cout << "ERROR: can not find panda_hand obj!" << std::endl;
    }
}

void CollisionChecker::constructFCLObjectRobot(const State& state)
{
    fk_.update(state);
    linksTf_ = fk_.getLinkTf();
    fclObjsRobotMap_.clear();

    for (const auto& link : linksTf_)
    {
        constructFCLObjectRobotLink(link.first, link.second);
    }
    for (const auto& obj : fclObjsRobotMap_)
        LOGD("fclObjsRobotMap_: %s", obj.first.data());
}

void CollisionChecker::constructFCLObjectRobotLink(const std::string name, const Eigen::Isometry3d& tf)
{
    std::vector<std::shared_ptr<fcl::CollisionObject>> fclObjects;
    const moveit::core::LinkModel* linkModel = robotModel_->getLinkModel(name);
    const std::vector<shapes::ShapeConstPtr> shapes = linkModel->getShapes();
    for (std::size_t j = 0; j < shapes.size(); j++)
    {
        std::shared_ptr<fcl::CollisionGeometry>  g = fclGeometriesRobot_[name][j];
        fcl::CollisionObject fcl_obj(g, eigen2Fcl(tf));
        auto fcl_objectPtr = std::make_shared<fcl::CollisionObject>(fcl_obj);
        fclObjects.push_back(fcl_objectPtr);
    }
    LOGD("constructFCLObj: %s", name.data());
    if (fclObjects.size() != 0)
        fclObjsRobotMap_.insert(std::make_pair(name, fclObjects));
}

void CollisionChecker::constructFCLGeometryRobot()
{
    fclGeometriesRobot_.clear();
    for (const auto& name : linkNames_)
    {
        fclGeometriesRobot_.insert(std::make_pair(name, constructFCLGeometryRobotLink(name)));
        LOGD("constructFCLGeometryRobot: %s", name.data());
    }
}

std::vector<std::shared_ptr<fcl::CollisionGeometry>> CollisionChecker::constructFCLGeometryRobotLink(const std::string& name)
{
    const moveit::core::LinkModel* link = robotModel_->getLinkModel(name);
    const std::vector<shapes::ShapeConstPtr> shapes = link->getShapes();
    std::vector<std::shared_ptr<fcl::CollisionGeometry>> geometries;
    
    for (std::size_t i = 0; i < shapes.size(); i ++)
    {
        std::shared_ptr<fcl::CollisionGeometry> g = constructFCLGeometry(shapes[i]);
        geometries.push_back(g);
        shapes[i]->print(std::cout);
    }
    LOGD("constructRobotGeometry: %s", name.data());
    return geometries;
}

void CollisionChecker::constructFCLObjectWorld(const collision_detection::WorldConstPtr& world)
{
    fclGeometriesWorld_.clear();
    fclObjsWorldMap_.clear();
    for (const auto& objectId : world->getObjectIds())
    {
        LOGD("constructFCLObjectWorld: %s", objectId.data());
        collision_detection::World::ObjectConstPtr object = world->getObject(objectId);
        std::vector<std::shared_ptr<fcl::CollisionObject>> fclObjects;
        for (std::size_t i = 0; i < object->shapes_.size(); i++)
        {
            std::shared_ptr<fcl::CollisionGeometry> g = constructFCLGeometry(object->shapes_[i]);
            fclGeometriesWorld_.push_back(g);
            object->shapes_[i]->print(std::cout);

            Eigen::Isometry3d tf = object->shape_poses_[i];
            fcl::CollisionObject fcl_obj(g, eigen2Fcl(tf));
            auto fcl_objectPtr = std::make_shared<fcl::CollisionObject>(fcl_obj);
            fclObjects.push_back(fcl_objectPtr);
        }
        fclObjsWorldMap_.insert(std::make_pair(objectId, fclObjects));
    }
}

std::shared_ptr<fcl::CollisionGeometry> CollisionChecker::constructFCLGeometry(const shapes::ShapeConstPtr& shape)
{
    // handle cases individually
    switch (shape->type)
    {
        case shapes::PLANE:
        {
            const shapes::Plane* p = static_cast<const shapes::Plane*>(shape.get());
            return std::make_shared<fcl::Plane>(p->a, p->b, p->c, p->d);
        }
        case shapes::SPHERE:
        {
            const shapes::Sphere* s = static_cast<const shapes::Sphere*>(shape.get());
            return std::make_shared<fcl::Sphere>(s->radius);
        }
        case shapes::BOX:
        {
            const shapes::Box* s = static_cast<const shapes::Box*>(shape.get());
            const double* size = s->size;
            return std::make_shared<fcl::Box>(size[0], size[1], size[2]);
        }
        case shapes::CYLINDER:
        {
            const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(shape.get());
            return std::make_shared<fcl::Cylinder>(s->radius, s->length);
        }
        case shapes::CONE:
        {
            const shapes::Cone* s = static_cast<const shapes::Cone*>(shape.get());
            return std::make_shared<fcl::Cone>(s->radius, s->length);
        }
        
        case shapes::MESH:
        {
            auto g = std::make_shared<fcl::BVHModel<fcl::OBBRSS>>();
            const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());
            if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
            {
                std::vector<fcl::Triangle> triIndices(mesh->triangle_count);
                for (unsigned int i = 0; i < mesh->triangle_count; ++i)
                triIndices[i] =
                    fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

                std::vector<fcl::Vec3f> points(mesh->vertex_count);
                for (unsigned int i = 0; i < mesh->vertex_count; ++i)
                points[i] = fcl::Vec3f(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);

                g->beginModel();
                g->addSubModel(points, triIndices);
                g->endModel();
            }
            return g;
        }
        case shapes::OCTREE:
        {
            const shapes::OcTree* g = static_cast<const shapes::OcTree*>(shape.get());
            return std::make_shared<fcl::OcTree>(g->octree);
        }
        default:
        LOGD("This shape type (%d) is not supported using FCL yet", (int)shape->type);
        return nullptr;
    }
}
}
