cmake_minimum_required(VERSION 3.13.0)
set(CMAKE_CXX_STANDARD 14)

project(laow_demo)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PythonLibs REQUIRED)

add_executable(laow_demo 
              src/main.cpp 
              src/calc_plan_stitch_trajectory.cpp 
              src/Cartesian2Frenet.cpp 
              src/Load_Roadmap.cpp 
              src/lqr_control.cpp 
              src/LQR_control.cpp 
              src/Obstacle.cpp 
              src/path_decision_task.cpp 
              src/Referenceline.cpp 
)

target_include_directories(laow_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PYTHON_INCLUDE_DIRS})

target_link_libraries(laow_demo 
  ${PYTHON_LIBRARIES}
  osqp::osqp
  OsqpEigen::OsqpEigen
)