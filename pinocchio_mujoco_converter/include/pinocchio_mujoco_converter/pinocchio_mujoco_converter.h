#pragma once

#include <Eigen/Core>
#include <mujoco_viewer/mujoco_viewer.h>
#include <string>

namespace pinocchio_mujoco_converter {
  // Full dimensions, mass and local axes match pinocchio_primitive_models.
  // Each primitive has a free joint and its origin at the center of mass.
  // These overloads initialize the viewer with a single primitive model.
  void createBox(const Eigen::Vector3d& size, const double& mass, mujoco_viewer::Viewer& viewer);
  void createSphere(const double& radius, const double& mass, mujoco_viewer::Viewer& viewer);
  void createCylinder(const double& radius, const double& length, const double& mass, mujoco_viewer::Viewer& viewer);

  // Add to an existing specification; compile it after adding all primitives.
  // Names must be unique within the specification. The returned body can be
  // positioned and its geometry customized before compilation.
  mjsBody* createBox(const Eigen::Vector3d& size, const double& mass, mjSpec& specification, const std::string& name = "box");
  mjsBody* createSphere(const double& radius, const double& mass, mjSpec& specification, const std::string& name = "sphere");
  mjsBody* createCylinder(const double& radius, const double& length, const double& mass, mjSpec& specification, const std::string& name = "cylinder");
}
