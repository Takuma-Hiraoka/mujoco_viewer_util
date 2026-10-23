#include <pinocchio_mujoco_converter/pinocchio_mujoco_converter.h>
#include <pinocchio_primitive_models/pinocchio_primitive_models.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
  try {
    const std::string shape = argc > 1 ? argv[1] : "all";
    const Eigen::Vector3d boxSize(0.3, 0.2, 0.4);
    const double radius = 0.15;
    const double length = 0.4;
    const double mass = 1.0;
    mujoco_viewer::Viewer viewer;
    if(shape == "all") {
      std::shared_ptr<mjSpec> specification(mj_makeSpec(), mj_deleteSpec);
      const std::vector<mjsBody*> bodies{
        pinocchio_mujoco_converter::createBox(boxSize, mass, *specification),
        pinocchio_mujoco_converter::createSphere(radius, mass, *specification),
        pinocchio_mujoco_converter::createCylinder(radius, length, mass, *specification)};
      for(std::size_t i = 0; i < bodies.size(); ++i) {
        bodies[i]->pos[0] = (double(i) - 1.0) * 0.6;
        bodies[i]->pos[2] = 0.3;
      }
      std::shared_ptr<mjModel> model(mj_compile(specification.get(), nullptr), mj_deleteModel);
      if(!model) throw std::runtime_error(mjs_getError(specification.get()));
      viewer.viewModel(model);
    } else {
      pinocchio::Model model;
      pinocchio::GeometryModel geometryModel;
      if(shape == "box") {
        pinocchio_primitive_models::createBox(boxSize, mass, model, geometryModel);
        pinocchio_mujoco_converter::createBox(boxSize, mass, viewer);
      } else if(shape == "sphere") {
        pinocchio_primitive_models::createSphere(radius, mass, model, geometryModel);
        pinocchio_mujoco_converter::createSphere(radius, mass, viewer);
      } else if(shape == "cylinder") {
        pinocchio_primitive_models::createCylinder(radius, length, mass, model, geometryModel);
        pinocchio_mujoco_converter::createCylinder(radius, length, mass, viewer);
      } else {
        throw std::invalid_argument("Usage: sample1 [all|box|sphere|cylinder]");
      }
      std::cout << shape << ": Pinocchio nv=" << model.nv
                << ", MuJoCo nv=" << viewer.model()->nv << std::endl;
    }
    while(viewer.isOpen()) {
      viewer.updateScene();
      viewer.drawScene();
      viewer.pollEvents();
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    return 0;
  } catch(const std::exception& error) {
    std::cerr << "sample1: " << error.what() << std::endl;
    return 1;
  }
}
