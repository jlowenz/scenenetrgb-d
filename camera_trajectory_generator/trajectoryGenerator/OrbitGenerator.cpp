#include "OrbitGenerator.hpp"

void sph2cart(const TooN::Vector<3>& sph, TooN::Vector<3>& cart)
{
  cart[0] = sph[2] * cos(sph[0]) * sin(sph[1]);
  cart[1] = sph[2] * cos(sph[1]);
  cart[2] = sph[2] * sin(sph[0]) * sin(sph[1]);
}

void cart2sph(const TooN::Vector<3>& cart, TooN::Vector<3>& sph)
{
  float radius = TooN::norm(cart);
  TooN::Vector<3> ncart = cart / radius;
  sph[1] = acos(ncart[1]);
  sph[0] = acos(ncart[0]/sin(sph[1]));
  sph[2] = radius;
}

OrbitGenerator::OrbitGenerator(CollisionInterfacePtr collision_checker,
                               FocusTargetInterfacePtr target)
  : collision_checker_(collision_checker),
    target_(target),
    re_((std::random_device())())
{
  
}

OrbitGenerator::~OrbitGenerator()
{
}

bool
OrbitGenerator::init(float init_radius, const Vector& scene_centroid)
{
  scene_centroid_ = scene_centroid;
  std::uniform_real_distribution<> unif(0,1.0);
  // choose some az between 0, 2pi
  orbit_pos_[0] = unif(re_) * 2 * M_PI;
  // choose some el between 0, pi/2
  orbit_pos_[1] = unif(re_) * M_PI/2.0;
  // start with the init radius
  orbit_pos_[2] = init_radius;
  orbit_vel_ = TooN::Zeros;
  on_sphere_ = true;
  
  target_->current_target(target_goal_);
  target_pos_ = scene_centroid_;
  target_vel_ = TooN::Zeros;
  sph2cart(orbit_pos_, camera_pos_);
  camera_pos_ += target_pos_;
  camera_vel_ = TooN::Zeros;

  num_attempts_ = 10;
  num_attempts_until_bounce_ = std::max(static_cast<int>(0.9 * num_attempts_), 1);

  max_orbit_speed_ = TooN::makeVector(0.5,0.2,0.1);
  orbit_accel_ = TooN::makeVector(0.7,0.5,0.05);
  min_radius_ = 0.7;
  max_radius_ = 2.5;
  
  initialized_ = true;

  return true;
}

Pose
OrbitGenerator::step(const float timestep)
{
  if (initialized_) {
    if (steps_ != 0) {
      update_target(timestep, target_pos_, target_vel_);
      update_camera(timestep, camera_pos_, camera_vel_);
    }
    steps_++;
    return calculate_pose();
  }
  std::cerr << "OrbitGenerator not intialized" << std::endl;
  return Pose();
}

void
OrbitGenerator::set_max_speed(float max_speed)
{
  max_speed_ = max_speed;  
}

void
OrbitGenerator::set_max_acceleration(float max_accel)
{
  max_acceleration_ = max_accel;
}

void
OrbitGenerator::set_max_orbit_speed(const Vector& orbit_speed)
{
  max_orbit_speed_ = orbit_speed;  
}

void
OrbitGenerator::set_orbit_accel(const Vector& orbit_accel)
{
  orbit_accel_ = orbit_accel;
}


void
OrbitGenerator::set_drag(float drag)
{
  drag_ = drag;
}

void
OrbitGenerator::set_radius_range(float maxradius, float minradius)
{
  max_radius_ = maxradius;
  min_radius_ = minradius;
}

Pose
OrbitGenerator::calculate_pose()
{
  return Pose(camera_pos_, target_pos_);
}


void
OrbitGenerator::manage_orbital_velocity(Vector& v, const Vector& p)
{
  // check the rates
  if (fabs(v[0]) > orbit_accel_[0]) {
    v[0] = std::copysign(orbit_accel_[0], v[0]);
  }
  if (fabs(v[1]) > orbit_accel_[1]) {
    v[1] = std::copysign(orbit_accel_[1], v[1]);
  }
  if (fabs(v[2]) > orbit_accel_[2]) {
    v[2] = std::copysign(orbit_accel_[2], v[2]);
  }
  
  // check on the poles (N/S) 
  if (p[1] < 1e-3 || p[1] > (M_PI - 1e-3)) {
    // reverse the phi direction
    v[1] = -v[1];
  }
  // check the radius
  if (p[2] < min_radius_ || p[2] > max_radius_) {
    v[2] = -v[2];
  }
}

void
OrbitGenerator::update_euclidean(float timestep, Vector& tmp_pos, Vector& tmp_vel,
                                 Vector& pos, Vector& vel)
{
  manage_orbital_velocity(tmp_vel, tmp_pos);
  tmp_pos += (timestep * tmp_vel);
  sph2cart(tmp_pos, pos);
  pos += target_pos_;
  vel = TooN::Zeros;

  std::cout << "orb vel: " << tmp_vel << std::endl
            << "orb pos: " << tmp_pos << std::endl
            << " target: " << target_pos_ << std::endl
            << "    pos: " << pos << std::endl;
}

  
void
OrbitGenerator::update_camera(float timestep, Vector& position, Vector& velocity)
{  
  // if it's on the sphere, then we do the orbital thing, if it's off the sphere,
  // we move towards the new sphere
  Vector sph;
  Vector delta = target_pos_ - position;
  cart2sph(delta, sph);
  float radius = TooN::norm(delta);
  if (target_changed_ == false &&
      radius - max_radius_ <= 0
      && radius - min_radius_ >= 0) {
    std::cout << "Orbiting... " << std::endl;
    // we're basically on the sphere
    // update the orbit
    Vector tmp_pos(orbit_pos_);
    Vector tmp_vel(orbit_vel_);
    Vector force = orbit_accel_;
    tmp_vel += (timestep * force);
    Vector tmp_cart;
    update_euclidean(timestep, tmp_pos, tmp_vel, tmp_cart, velocity);
    
    if (!collision_checker_->collided(position, tmp_cart)) {
      orbit_pos_ = tmp_pos;
      orbit_vel_ = tmp_vel;
    } else {      
      tmp_vel[0] = -tmp_vel[0];
      while (collision_checker_->collided(position, tmp_cart)) {
        update_euclidean(timestep, tmp_pos, tmp_vel, tmp_cart, velocity);
      }
      orbit_pos_ = tmp_pos;
      orbit_vel_ = tmp_vel;
    }

  } else {
    std::cout << "Moving to target..." << std::endl;
    float vel_squared = TooN::norm_sq(velocity);
    Vector drag = TooN::Zeros;
    if (vel_squared > 1.0e-12) {
      drag = -TooN::unit(velocity) * (0.5 * 1.204 * 0.09 * drag_ * vel_squared);
    }
    // we move towards the sphere using the Euclidean vector towards the target
    Vector tmp_pos(position);
    Vector tmp_vel(velocity);
    Vector force = max_acceleration_ * TooN::unit(target_pos_ - tmp_pos);
    tmp_vel += (force + drag) * timestep;
    float scale = TooN::norm(tmp_vel) / max_speed_;
    if (scale > 1.0) {
      tmp_vel /= scale;
    }
    tmp_pos += timestep * tmp_vel;
    if (!collision_checker_->collided(position, tmp_pos)) {
      velocity = tmp_vel;
      position = tmp_pos;
      cart2sph(position - target_pos_, orbit_pos_);
      orbit_vel_ = TooN::Zeros;
    }
  }
}

void
OrbitGenerator::update_target(float timestep, Vector& position, Vector& velocity)
{
  float vel_squared = TooN::norm_sq(velocity);
  Vector drag = TooN::Zeros;
  if (vel_squared > 1.0e-12) {
    drag = -TooN::unit(velocity) * (0.5 * 1.204 * 0.09 * drag_ * vel_squared);
  }

  Vector new_target;
  target_->current_target(new_target);
  if (TooN::norm(new_target - target_goal_) > 0.01) {
    target_goal_ = new_target;
    target_changed_ = true;
  }

  if (TooN::norm(target_pos_ - target_goal_) < 0.01) {
    target_changed_ = false;
  }
  
  Vector tmp_pos(position);
  Vector tmp_vel(velocity);
  Vector applied_force;
  applied_force = max_acceleration_ * TooN::unit(target_goal_ - target_pos_);;    
  
  tmp_vel += (timestep * (drag + applied_force));
  float scale_factor = TooN::norm(tmp_vel) / max_speed_;
  if (scale_factor > 1.0) {
    tmp_vel /= scale_factor;    
  }
  tmp_pos += (timestep * tmp_vel);
  velocity = tmp_vel;
  position = tmp_pos;

}
