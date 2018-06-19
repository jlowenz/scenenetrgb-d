#include "OrbitGenerator.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

namespace bg = boost::geometry;
namespace cs = bg::cs;

struct sph
{
  TooN::Vector<3> model_;
  double theta, phi, radius;
  sph(const TooN::Vector<3>& mdl) : model_(mdl), theta(mdl[0]), phi(mdl[1]), radius(mdl[2]) {}
  operator const TooN::Vector<3>&() {
    model_[0] = theta;
    model_[1] = phi;
    model_[2] = radius;
    return model_;
  }
};

BOOST_GEOMETRY_REGISTER_POINT_3D(TooN::Vector<3>, double, cs::cartesian, operator[](0), operator[](2), operator[](1));
BOOST_GEOMETRY_REGISTER_POINT_3D(sph, double, cs::spherical<bg::radian>, theta, phi, radius);

void sph2cart(const TooN::Vector<3>& sphere, TooN::Vector<3>& cart)
{  
  // cart[0] = sph[2] * cos(sph[0]) * sin(sph[1]);
  // cart[1] = sph[2] * cos(sph[1]);
  // cart[2] = sph[2] * sin(sph[0]) * sin(sph[1]);
  sph s(sphere);
  bg::transform(s, cart);
}

void cart2sph(const TooN::Vector<3>& cart, TooN::Vector<3>& sphere)
{
  // float radius = TooN::norm(cart);
  // //TooN::Vector<3> ncart = cart / radius;
  sph s(sphere);
  std::cout << "sph:0 " << sphere[0] << " " << sphere[1] << " " << sphere[2] << std::endl;
  bg::transform(cart, s);
  sphere = s;
  sphere[0] = (sphere[0] < 0) ? (2*M_PI+sphere[0]) : sphere[0]; // normalize 0->2pi
  // sph[1] = acos(cart[1]/radius);
  // sph[0] = atan2(cart[2],cart[0]);
  // sph[2] = radius;
  std::cout << "sph:1 " << sphere[0] << " " << sphere[1] << " " << sphere[2] << std::endl;
}

OrbitGenerator::OrbitGenerator(CollisionInterfacePtr collision_checker,
                               FocusTargetInterfacePtr target,
                               int num_poses)
  : collision_checker_(collision_checker),
    target_(target),
    re_((std::random_device())()),
    max_poses_(num_poses),
    curr_pose_(0)
{
  
}

OrbitGenerator::~OrbitGenerator()
{
}

void
OrbitGenerator::orbit2cam()
{
  sph2cart(orbit_pos_, camera_pos_);
  camera_pos_ += target_pos_;
  //camera_vel_ = TooN::Zeros;
}

bool
OrbitGenerator::init(float min_radius, float max_radius, float speed, float accel)
{
  max_radius_ = max_radius;
  min_radius_ = min_radius;
  max_speed_ = speed;
  max_accel_ = accel;
  float init_radius = unif_(re_)*(max_radius - min_radius) + min_radius;

  // orbit_pos_ represents the current point on the sphere
  // choose some az between 0, 2pi
  orbit_pos_[0] = unif_(re_) * 2 * M_PI;
  // choose some el between 0, pi/2
  orbit_pos_[1] = unif_(re_) * M_PI/2.0;
  // start with the init radius
  orbit_pos_[2] = init_radius;
  orbit_vel_ = 0;
  state_ = ChangeRadius;

  next_waypoint();
  
  target_->current_target(target_goal_);
  target_pos_ = target_goal_;
  target_vel_ = TooN::Zeros;

  // camera_pos_ represents the actual world camera coords
  orbit2cam();
  camera_vel_ = TooN::Zeros;

  num_attempts_ = 10;
  num_attempts_until_bounce_ = std::max(static_cast<int>(0.9 * num_attempts_), 1);

  // max_orbit_speed_ = TooN::makeVector(0.5,0.2,0.1);
  // orbit_accel_ = TooN::makeVector(0.7,0.5,0.05);
  // min_radius_ = 0.7;
  // max_radius_ = 2.5;
  
  initialized_ = true;

  return true;
}

double
OrbitGenerator::geodesic_distance()
{
  double eq = M_PI/2.0;
  double phi1 = eq - orbit_pos_[1];
  double theta1 = orbit_pos_[0];
  double phi2 = eq - orbit_wp_pos_[1];
  double theta2 = orbit_wp_pos_[0];
  double dx = cos(phi2)*cos(theta2) - cos(phi1)*cos(theta1);
  double dy = cos(phi2)*sin(theta2) - cos(phi1)*sin(theta1);
  double dz = sin(phi2) - sin(phi1);
  double C = sqrt(dx*dx + dy*dy + dz*dz);
  double ds = 2.0 * asin(C/2.0);
  return orbit_pos_[2] * ds;
}

bool near(double a, double b, double tol = 1e-3)
{
  return fabs(a-b) < tol;
}

bool near(TooN::Vector<3>& a, TooN::Vector<3>& b, double tol = 1e-3)
{
  double d = TooN::norm(a-b);
  return d < tol;
}

void
OrbitGenerator::next_waypoint()
{
  curr_pose_++;
  // find the next sphere waypoint
  orbit_wp_pos_[0] = unif_(re_) * 2 * M_PI;
  orbit_wp_pos_[1] = unif_(re_) * M_PI/2.0;
  orbit_wp_pos_[2] = unif_(re_)*(max_radius_-min_radius_)+min_radius_;
  state_ = ChangeRadius;
}

Pose
OrbitGenerator::step(const float timestep)
{
  if (state_ == ChangeRadius) {
    if (!near(orbit_wp_pos_[2],orbit_pos_[2])) {
      state_ = ChangeRadius;      
    } else {
      state_ = OnSphere;
      // compute the total geodesic distance
      Vector a, b;
      sph2cart(orbit_pos_, a);
      sph2cart(orbit_wp_pos_, b);
      
      dist_ = geodesic_distance();
    }
  }
  
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
  max_accel_ = max_accel;
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
  std::cout << "Orbiting... " << std::endl;
  // we're basically on the sphere
  // update the orbit

  Vector tmp_pos;
  float tmp_vel = orbit_vel_;
  sph2cart(orbit_pos_, tmp_pos);
  std::cout << "tmp_pos: " << tmp_pos << std::endl;
  
  if (state_ == ChangeRadius) {
    double diff = orbit_wp_pos_[2] - orbit_pos_[2];
    
    // move along the normal
    tmp_vel = std::min<double>(tmp_vel + timestep*2*max_accel_,
                               std::min<double>(max_speed_,fabs(diff)+1e-2));
    Vector normal = TooN::unit(tmp_pos);    
    velocity = normal * std::copysign(tmp_vel,diff);
    tmp_pos = tmp_pos + timestep * velocity;
    // object to world
    position = tmp_pos + target_pos_;

    orbit_vel_ = tmp_vel;
    cart2sph(tmp_pos, orbit_pos_);
  } else if (state_ == OnSphere) {
    Vector tmp_wp;
    sph2cart(orbit_wp_pos_, tmp_wp);
    double diff = geodesic_distance();
    // we will do the simplest thing possible here... and estimate the next pose
    // based on the tangent to the sphere.
    Vector norm = TooN::unit(tmp_pos);
    Vector to_plane = norm*(tmp_pos - tmp_wp) * norm;
    Vector tangent = TooN::unit(tmp_wp + to_plane - tmp_pos); 
    std::cout << "tangent: " << tangent << std::endl;
    tmp_vel = std::min<double>(std::min<double>(max_speed_,diff+1e-4),
                               tmp_vel + timestep*max_accel_*0.5);
    tmp_pos = TooN::unit(tmp_vel * tangent + tmp_pos) * orbit_pos_[2];
    std::cout << "tmp_pos: " << tmp_pos << std::endl;
    position = tmp_pos + target_pos_;

    orbit_vel_ = tmp_vel;
    cart2sph(tmp_pos, orbit_pos_);
  }

  Vector wp;
  sph2cart(orbit_wp_pos_, wp);
  // find out if we are near the wp
  if (near(wp, tmp_pos, 0.002)) {
    next_waypoint();
    state_ = ChangeRadius;
  } else {
    std::cout << std::endl;
    std::cout << "tmp_vel     : " << tmp_vel << std::endl;
    std::cout << "not near yet: " << orbit_pos_ << std::endl;
    std::cout << "orbit_wp    : " << orbit_wp_pos_ << std::endl;
    std::cout << "tmp_pos     : " << tmp_pos << std::endl;
    std::cout << "wp          : " << wp << std::endl;
    std::cout << "geo_dist    : " << geodesic_distance() << std::endl;
    std::cout << "euc_obj_dist: " << TooN::norm(wp-tmp_pos) << std::endl;
    std::cout << "target_pose : " << target_pos_ << std::endl;
    std::cout << "state       : " << state_ << std::endl;
  }
}

void
OrbitGenerator::update_target(float timestep, Vector& position, Vector& velocity)
{
  position = target_pos_;
  velocity = target_vel_;
}
