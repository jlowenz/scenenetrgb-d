#pragma once

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <memory>

struct Pose
{
  TooN::Vector<3> target;
  TooN::Vector<3> camera;

  Pose() {}
  Pose(const TooN::Vector<3>& cam, const TooN::Vector<3>& look)
    : camera(cam), target(look) {}
  Pose(const Pose& other)
    : camera(other.camera), target(other.target) {}

  Pose& operator=(const Pose& other) {
    target = other.target;
    camera = other.camera;
    return *this;
  }
};

struct ITrajectoryGenerator
{
  virtual Pose calculate_pose() = 0;
  virtual Pose step(const float timestep) = 0;
};

typedef std::shared_ptr<ITrajectoryGenerator> TrajGenPtr;

class CollisionInterface {
public:
  // True implies a collision, false no collision
  virtual bool collided(const TooN::Vector<3,float> position, const TooN::Vector<3,float> next_position) = 0;
};

typedef std::shared_ptr<CollisionInterface> CollisionInterfacePtr;

class FocusTargetInterface {
public:
  virtual void step(float delta) = 0;
  virtual void current_target(TooN::Vector<3>& target) = 0;
  //virtual void next_target() = 0;
};

typedef std::shared_ptr<FocusTargetInterface> FocusTargetInterfacePtr;

