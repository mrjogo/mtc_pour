#pragma once

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_task_constructor_msgs/msg/solution.hpp>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_extents.h>
#include <geometric_shapes/shape_operations.h>

#include <shape_msgs/msg/mesh.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

namespace mtc_pour {

void collisionObjectFromResource(moveit_msgs::msg::CollisionObject &msg,
                                 const std::string &id,
                                 const std::string &resource) {
  msg.meshes.resize(1);

  // load mesh
  const Eigen::Vector3d scaling(1, 1, 1);
  shapes::Shape *shape = shapes::createMeshFromResource(resource, scaling);
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(shape, shape_msg);
  msg.meshes[0] = boost::get<shape_msgs::msg::Mesh>(shape_msg);

  // set pose
  msg.mesh_poses.resize(1);
  msg.mesh_poses[0].orientation.w = 1.0;

  // fill in details for MoveIt
  msg.id = id;
  msg.operation = moveit_msgs::msg::CollisionObject::ADD;
}

double computeMeshHeight(const shape_msgs::msg::Mesh &mesh) {
  double x, y, z;
  geometric_shapes::getShapeExtents(mesh, x, y, z);
  return z;
}

void cleanup() {
  moveit::planning_interface::PlanningSceneInterface psi;

  {
    std::map<std::string, moveit_msgs::msg::AttachedCollisionObject>
        attached_objects = psi.getAttachedObjects({"bottle"});
    if (attached_objects.count("bottle") > 0) {
      attached_objects["bottle"].object.operation =
          moveit_msgs::msg::CollisionObject::REMOVE;
      psi.applyAttachedCollisionObject(attached_objects["bottle"]);
    }
  }

  {
    std::vector<std::string> names = psi.getKnownObjectNames();

    std::vector<moveit_msgs::msg::CollisionObject> objs;
    for (std::string &obj :
         std::vector<std::string>{"table", "glass", "bottle"}) {
      if (std::find(names.begin(), names.end(), obj) != names.end()) {
        objs.emplace_back();
        objs.back().id = obj;
        objs.back().operation = moveit_msgs::msg::CollisionObject::REMOVE;
      }
    }

    psi.applyCollisionObjects(objs);
  }
}

void setupTable(std::vector<moveit_msgs::msg::CollisionObject> &objects,
                const geometry_msgs::msg::PoseStamped &tabletop_pose) {
  // add table
  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header = tabletop_pose.header;
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives[0].dimensions.resize(3);
  table.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = .5;
  table.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 1.0;
  table.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = .1;

  table.primitive_poses.resize(1);
  table.primitive_poses[0] = tabletop_pose.pose;
  table.primitive_poses[0].orientation.w = 1;
  table.primitive_poses[0].position.z -=
      (table.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] / 2);

  objects.push_back(std::move(table));
}

void setupObjects(
    std::vector<moveit_msgs::msg::CollisionObject> &objects,
    const geometry_msgs::msg::PoseStamped &bottle_pose,
    const geometry_msgs::msg::PoseStamped &glass_pose,
    const std::string &bottle_mesh = "package://mtc_pour/meshes/bottle.stl",
    const std::string &glass_mesh = "package://mtc_pour/meshes/glass.stl") {

  {
    // add bottle
    objects.emplace_back();
    collisionObjectFromResource(objects.back(), "bottle", bottle_mesh);
    objects.back().header = bottle_pose.header;
    objects.back().mesh_poses[0] = bottle_pose.pose;

    // The input pose is interpreted as a point *on* the table
    objects.back().mesh_poses[0].position.z +=
        computeMeshHeight(objects.back().meshes[0]) / 2 + .002;
  }

  {
    // add glass
    objects.emplace_back();
    collisionObjectFromResource(objects.back(), "glass", glass_mesh);
    objects.back().header = glass_pose.header;
    objects.back().mesh_poses[0] = glass_pose.pose;
    // The input pose is interpreted as a point *on* the table
    objects.back().mesh_poses[0].position.z +=
        computeMeshHeight(objects.back().meshes[0]) / 2 + .002;
  }
}

} // namespace mtc_pour
