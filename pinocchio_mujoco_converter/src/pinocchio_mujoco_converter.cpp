#include <pinocchio_mujoco_converter/pinocchio_mujoco_converter.h>

#include <cmath>
#include <memory>
#include <stdexcept>

namespace pinocchio_mujoco_converter {
  namespace {
    void checkPositive(double value) {
      if(!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("Primitive dimensions and mass must be finite and positive");
    }

    mjsBody* createPrimitive(mjSpec& specification, const std::string& name,
                            mjtGeom type, const Eigen::Vector3d& size, double mass) {
      checkPositive(mass);
      if(name.empty() || mjs_findBody(&specification, name.c_str()))
        throw std::invalid_argument("Primitive body name is empty or already exists: " + name);
      mjsBody* world = mjs_findBody(&specification, "world");
      if(!world) throw std::runtime_error("MuJoCo world body was not found");
      mjsBody* body = mjs_addBody(world, nullptr);
      mjs_setName(body->element, name.c_str());
      mjsJoint* joint = mjs_addFreeJoint(body);
      mjs_setName(joint->element, (name + "_base_joint").c_str());
      mjsGeom* geometry = mjs_addGeom(body, nullptr);
      mjs_setName(geometry->element, (name + "_geom").c_str());
      geometry->type = type;
      for(int i = 0; i < 3; ++i) geometry->size[i] = size[i];
      geometry->mass = mass;
      geometry->rgba[0] = 0.3f;
      geometry->rgba[1] = 0.6f;
      geometry->rgba[2] = 0.9f;
      geometry->rgba[3] = 1.0f;
      return body;
    }

    void viewSpecification(mjSpec& specification, mujoco_viewer::Viewer& viewer) {
      std::shared_ptr<mjModel> model(mj_compile(&specification, nullptr), mj_deleteModel);
      if(!model) throw std::runtime_error(mjs_getError(&specification));
      viewer.viewModel(model);
    }
  }

  mjsBody* createBox(const Eigen::Vector3d& size, const double& mass, mjSpec& specification, const std::string& name) {
    for(int i = 0; i < 3; ++i) checkPositive(size[i]);
    return createPrimitive(specification, name, mjGEOM_BOX, size * 0.5, mass);
  }

  mjsBody* createSphere(const double& radius, const double& mass, mjSpec& specification, const std::string& name) {
    checkPositive(radius);
    return createPrimitive(specification, name, mjGEOM_SPHERE, Eigen::Vector3d(radius, 0, 0), mass);
  }

  mjsBody* createCylinder(const double& radius, const double& length, const double& mass, mjSpec& specification, const std::string& name) {
    checkPositive(radius);
    checkPositive(length);
    return createPrimitive(specification, name, mjGEOM_CYLINDER, Eigen::Vector3d(radius, length * 0.5, 0), mass);
  }

  void createBox(const Eigen::Vector3d& size, const double& mass, mujoco_viewer::Viewer& viewer) {
    std::shared_ptr<mjSpec> specification(mj_makeSpec(), mj_deleteSpec);
    createBox(size, mass, *specification);
    viewSpecification(*specification, viewer);
  }

  void createSphere(const double& radius, const double& mass, mujoco_viewer::Viewer& viewer) {
    std::shared_ptr<mjSpec> specification(mj_makeSpec(), mj_deleteSpec);
    createSphere(radius, mass, *specification);
    viewSpecification(*specification, viewer);
  }

  void createCylinder(const double& radius, const double& length, const double& mass, mujoco_viewer::Viewer& viewer) {
    std::shared_ptr<mjSpec> specification(mj_makeSpec(), mj_deleteSpec);
    createCylinder(radius, length, mass, *specification);
    viewSpecification(*specification, viewer);
  }
}
