#ifndef __KFCMD_HYPO3PHOTONS_HPP__
#define __KFCMD_HYPO3PHOTONS_HPP__
#include "Hypothesis.hpp"

namespace KFCmd {
class Hypo3Photons : public Hypothesis {
 public:
  Hypo3Photons(double, long = 20, double = 1.e-3);
  virtual ~Hypo3Photons();
};
}  // namespace KFCmd

#endif
