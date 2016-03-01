#pragma once

#include <string>

#include "Pose.h"

namespace core {
class Gesture {
 public:
  enum Type { unknown };

  Gesture(Type type = unknown);
  Gesture(const std::shared_ptr<Pose>& pose, Type type = unknown);
  virtual ~Gesture() {}

  bool operator==(const Gesture& gesture) const;
  bool operator!=(const Gesture& gesture) const;

  virtual std::string toString() const;

  std::shared_ptr<Pose> AssociatedPose() const;

  std::string toDescriptiveString() const;

 protected:
  const std::shared_ptr<Pose> associated_pose_;

 private:
  const Type type_;
};
}
