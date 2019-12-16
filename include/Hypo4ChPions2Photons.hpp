#ifndef __KFCMD_HYPO4CHPIONS2PHOTONS_HPP__
#define __KFCMD_HYPO4CHPIONS2PHOTONS_HPP__
#include "Hypothesis.hpp"

namespace KFCmd {
class Hypo4ChPions2Photons : public Hypothesis {
 public:
  Hypo4ChPions2Photons(double, double, long = 20, double = 1.e-3);
  virtual ~Hypo4ChPions2Photons();
};
}  // namespace KFCmd

#endif
