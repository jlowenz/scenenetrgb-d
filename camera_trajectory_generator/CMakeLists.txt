cmake_minimum_required(VERSION 3.10.0)

project(CameraTrajectoryGenerator)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

set(CMAKE_CXX_STANDARD 14)

set(OpenGL_GL_PREFERENCE "LEGACY")
set(_OPENGL_LIB_PATH "/usr/lib/nvidia-390")

find_package(CUDA REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CVD REQUIRED)
find_package(GLUT REQUIRED)
find_package(Pangolin REQUIRED)
FIND_PACKAGE(ASSIMP REQUIRED)
find_package(Boost REQUIRED)

message("Assimp Lib:" ${ASSIMP_LIBRARY})

INCLUDE_DIRECTORIES(
  ${Pangolin_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${CVD_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
  ${ASSIMP_INCLUDE}
  ${Boost_INCLUDE_DIRS})

LINK_LIBRARIES(
  ${Pangolin_LIBRARY}
  ${CVD_LIBRARY}
  ${ASSIMP_LIBRARY}
  -lGLEW
  -lglut
  -lGLU
  -lGL
  -lboost_system
  -lboost_filesystem
  )

SET(ASSIMPOBJLOADER_SOURCES
  ./assimpobjloader/assimp_obj_loader.h)

ADD_EXECUTABLE(room_camera_intersection
  ./trajectoryGenerator/OrbitGenerator.hpp
  ./trajectoryGenerator/OrbitGenerator.cpp
  ./trajectoryGenerator/TrajectoryGenerator.hpp
  ./trajectoryGenerator/TrajectoryGenerator.cpp
  camera_trajectory_generator.cpp)
