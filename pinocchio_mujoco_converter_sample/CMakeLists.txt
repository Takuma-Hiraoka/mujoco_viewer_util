cmake_minimum_required(VERSION 3.16)
project(pinocchio_mujoco_converter_sample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)

set(dependencies
  pinocchio_primitive_models
  pinocchio_mujoco_converter
)

foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

add_executable(sample1 src/sample1.cpp)
ament_target_dependencies(sample1 ${dependencies})

install(TARGETS sample1 DESTINATION lib/${PROJECT_NAME})

ament_package()
