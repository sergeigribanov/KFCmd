#ifndef __KFCMD_HYPO3CHPIONSKMINUS_HPP__
#define __KFCMD_HYPO3CHPIONSKMINUS_HPP__
#include "Hypothesis.hpp"

namespace KFCmd {
class Hypo3ChPionsKMinus : public Hypothesis {
 public:
  Hypo3ChPionsKMinus(double, double, long = 20, double = 1.e-3);
  virtual ~Hypo3ChPionsKMinus();
};
}  // namespace KFCmd

#endif
