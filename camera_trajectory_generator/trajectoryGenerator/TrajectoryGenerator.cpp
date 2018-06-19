/*
 * This file is part of SceneNet RGB-D.
 *
 * Copyright (C) 2017 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is SemanticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/semantic-fusion/scenenet-rgbd-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "TrajectoryGenerator.hpp"

#include <iostream>


TrajectoryGenerator::TrajectoryGenerator(std::shared_ptr<CollisionInterface> collision_checker,
                                         std::shared_ptr<FocusTargetInterface> target,
                                         const int num_attempts)
  : collision_checker_(collision_checker)
  , target_(target)
  , random_eng_((std::random_device())())
  , num_attempts_(num_attempts)
  , num_attempts_until_bounce_(static_cast<int>(0.9 * num_attempts_) + 1)
  , max_speed_(5.0)                    // m/s
  , max_acceleration_(2.5)             // m/s^2
  , drag_(0.1)                         // drag coefficient (no units)
  , initialised_(false) 
  , steps_(0) {}

bool
TrajectoryGenerator::init(const Vector initial_pose, const Vector lookat_pose) {
  min_radius_ = 0.5;
  max_radius_ = 2.75;

  position_ = initial_pose;
  velocity_ = TooN::Zeros;
  // THIS CHECK CAN NO LONGER BE DONE IN THE SAME WAY
  //if (!collision_checker_->collided(position_)) {
  target_position_ = lookat_pose;
  target_velocity_ = TooN::Zeros;
  initialised_ = true;
  return true;
  //  initialised_ = !collision_checker_->collided(target_position_);
  //  return initialised_;
  //}
  //return false;
}

Pose
TrajectoryGenerator::step(const float timestep) {
  if (initialised_) {
    if (steps_ != 0) {
      UpdateObject(timestep,position_,velocity_);
      UpdateObject(timestep,target_position_,target_velocity_, true);
    }
    steps_++;
    return calculate_pose();
  } 
  std::cerr << "TrajectoryGenerator improperly initialised" << std::endl;
  return Pose();
}

bool
TrajectoryGenerator::radius_limited(const TooN::Vector<3>& new_pos)
{
  if (target_) {
    Vector look_at;
    target_->current_target(look_at);
    float dist = TooN::norm(new_pos - look_at);
    if (dist < min_radius_) {
      target_field_ = TooN::unit(new_pos - look_at);
      // push it away
    } else if (dist > max_radius_) {
      target_field_ = TooN::unit(look_at - new_pos);
      // pull it closer
    } else {
      // wander
      target_field_ = TooN::Zeros;
      return false;
    }
    return true;
  }
  return false;
}

void
TrajectoryGenerator::UpdateObject(const float timestep, Vector& position,
                                  Vector& velocity, bool is_target)
{
  float velocity_squared = TooN::norm_sq(velocity);
  Vector drag = TooN::Zeros;

  if (velocity_squared > 1.0e-12)
  {
    // Drag is modelled roughly on air at 20 degrees with a 30cm radius ball
    drag = -TooN::unit(velocity) * (0.5 * 1.204 * 0.09 * drag_ * velocity_squared);
  }

  // Simple euler integration is sufficient and performed until no collision
  for (int attempts = 0; attempts < num_attempts_; ++attempts)
  {
    Vector temporary_position(position);
    Vector temporary_velocity(velocity);    
    Vector applied_force;
    if (is_target) {
      applied_force = max_acceleration_ * target_unit_vector();
    } else {
      applied_force = max_acceleration_ * camera_unit_vector();
    }
    // The mass of the trajectory object is implicitly 1Kg
    temporary_velocity += (timestep * (drag + applied_force));
    // We scale the velocity to be within the maximum allowed speed
    float scale_factor = TooN::norm(temporary_velocity) / max_speed_;
    if (scale_factor > 1.0) {
      temporary_velocity /= scale_factor;
    }
    // Update the proposed object position and check for collisions
    temporary_position += (timestep * temporary_velocity);

    if (!is_target) {    
      if (!collision_checker_->collided(position,temporary_position)) {
        velocity = temporary_velocity;
        position = temporary_position;
        break;
      }
      // Reverse the direction of velocity if many collisions one way (i.e. bounce)
      if (attempts == num_attempts_until_bounce_) {
        velocity *= -1.0;
      }
      // If we have exhausted our attempts the trajectory stops dead
      if (attempts >= num_attempts_ - 1) {
        velocity = TooN::Zeros;
      }
    } else {
      position = temporary_position;
      velocity = temporary_velocity;
    }
  }

}

Pose TrajectoryGenerator::calculate_pose() {
  // NOTE Here is a place where tilt could potentially be introduced
  //std::cout<<"Pose:"<<position_[0]<<" "<<position_[1]<<" "<<position_[2]<<std::endl;
  //std::cout<<"Look at:"<<target_position_[0]<<" "<<target_position_[1]<<" "<<target_position_[2]<<std::endl;
  return Pose(position_,target_position_);
}

TrajectoryGenerator::Vector
TrajectoryGenerator::random_unit_vector() {
  std::normal_distribution<float> normal_dist(0,1);
  Vector vector;
  for (int index = 0; index < 3; ++index) {
    vector[index] = normal_dist(random_eng_);
  }
  return TooN::unit(vector);
}

TrajectoryGenerator::Vector
TrajectoryGenerator::camera_unit_vector()
{
  if (radius_limited(position_)) {
    return TooN::unit(0.5 * random_unit_vector() + 0.5 * target_field_);
  } else {
    return random_unit_vector();
  }
}

TrajectoryGenerator::Vector
TrajectoryGenerator::target_unit_vector() {
  if (target_) {
    Vector look_at;
    target_->current_target(look_at);
    Vector delta = TooN::unit(look_at - target_position_);
    if (!std::isfinite(delta[0])) {
      return TooN::Zeros;
    } else {
      return TooN::unit(0.5 * delta + 0.5 * random_unit_vector());
    }
  } else {
    return random_unit_vector();
  }
}
