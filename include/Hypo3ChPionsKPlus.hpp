#ifndef __KFCMD_HYPO3CHPIONSKPLUS_HPP__
#define __KFCMD_HYPO3CHPIONSKPLUS_HPP__
#include "Hypothesis.hpp"

namespace KFCmd {
class Hypo3ChPionsKPlus : public Hypothesis {
 public:
  Hypo3ChPionsKPlus(double, double, long = 20, double = 1.e-3);
  virtual ~Hypo3ChPionsKPlus();
};
}  // namespace KFCmd

#endif
