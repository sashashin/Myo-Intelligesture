#include "Gesture.h"

namespace core {
Gesture::Gesture(Type type)
    : type_(type), associated_pose_(new Pose(Pose::rest)) {}

Gesture::Gesture(const std::shared_ptr<Pose>& pose, Type type)
    : type_(type), associated_pose_(pose) {}

bool Gesture::operator==(const Gesture& gesture) const {
  return this->toString() == gesture.toString();
}

bool Gesture::operator!=(const Gesture& gesture) const {
  return this->toString() != gesture.toString();
}

std::string Gesture::toString() const { return "unknown"; }

std::shared_ptr<Pose> Gesture::AssociatedPose() const {
  return associated_pose_;
}

std::string Gesture::toDescriptiveString() const {
  return this->toString() + ": " + associated_pose_->toString();
}

std::ostream& operator<<(std::ostream& os, const Gesture& gesture) {
  return os << gesture.toString();
}
}