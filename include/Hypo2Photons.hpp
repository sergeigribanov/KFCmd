#ifndef __KFCMD_HYPO2PHOTONS_HPP__
#define __KFCMD_HYPO2PHOTONS_HPP__
#include "Hypothesis.hpp"

namespace KFCmd {
class Hypo2Photons : public Hypothesis {
 public:
  Hypo2Photons(double, long = 20, double = 1.e-3);
  virtual ~Hypo2Photons();
};
}  // namespace KFCmd

#endif
