#pragma once

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <memory>

class CollisionInterface {
public:
  // True implies a collision, false no collision
  virtual bool collided(const TooN::Vector<3,float> position, const TooN::Vector<3,float> next_position) = 0;
};

typedef std::shared_ptr<CollisionInterface> CollisionInterfacePtr;
