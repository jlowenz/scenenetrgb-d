#pragma once

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <memory>

#include <random>
#include <utility>

#include "interfaces.hpp"


// Orbit generator in spherical coords

class OrbitGenerator : public ITrajectoryGenerator {
public:
  typedef TooN::Vector<3> Vector;
  
  OrbitGenerator(CollisionInterfacePtr collision_checker,
                 FocusTargetInterfacePtr target);
  ~OrbitGenerator();

  bool init(float init_radius, const Vector& scene_centroid);
  Pose step(const float timestep = 0.025);
  void set_max_speed(float max_speed);
  void set_max_acceleration(float max_accel);
  void set_radius_range(float maxradius, float minradius);
  void set_drag(float drag);
  void set_max_orbit_speed(const Vector& orbit_speed);
  void set_orbit_accel(const Vector& orbit_accel);
  
  Pose calculate_pose();

private:
  void update_target(float timestep, Vector& position, Vector& velocity);
  void update_camera(float timestep, Vector& position, Vector& velocity);
  void manage_orbital_velocity(Vector& v, const Vector& p);
  void update_euclidean(float timestep, Vector& tmp_pos, Vector& tmp_vel,
                        Vector& pos, Vector& vel);

  CollisionInterfacePtr collision_checker_;
  FocusTargetInterfacePtr target_;

  std::default_random_engine re_;

  Vector scene_centroid_;
  
  float max_speed_;
  float max_acceleration_;

  bool on_sphere_;
  // spherical accelerations
  Vector max_orbit_speed_;
  Vector orbit_accel_;
  float min_radius_;
  float max_radius_;
  
  float drag_;
  bool initialized_;
  int steps_;

  int num_attempts_;
  int num_attempts_until_bounce_;
  
  // 0      1    2
  // az     el   height
  // theta, phi, radius
  Vector orbit_pos_;
  Vector orbit_vel_;

  bool target_changed_;
  Vector target_goal_;
  Vector target_pos_;
  Vector target_vel_;

  Vector camera_pos_;
  Vector camera_vel_;
  
};
