#ifndef __KFCMD_HYPO4CHPIONS_HPP__
#define __KFCMD_HYPO4CHPIONS_HPP__
#include "Hypothesis.hpp"

namespace KFCmd {
class Hypo4ChPions : public Hypothesis {
 public:
  Hypo4ChPions(double, double, long = 20, double = 1.e-3);
  virtual ~Hypo4ChPions();
};
}  // namespace KFCmd

#endif
