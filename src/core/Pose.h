#pragma once

#include <string>
#include <iostream>
#include <memory>
#include <myo/myo.hpp>

namespace core {
class Pose {
 public:
  enum Type { rest, fist, waveIn, waveOut, fingersSpread, doubleTap, unknown };

  Pose(Type type = unknown);
  Pose(const myo::Pose& pose);
  virtual ~Pose() {}

  bool operator==(const Pose& pose) const;
  bool operator!=(const Pose& pose) const;

  virtual std::string toString() const;

 private:
  Type type_;
};
}
