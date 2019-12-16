#ifndef __KFCMD_PIPLUSMESON_HPP__
#define __KFCMD_PIPLUSMESON_HPP__
#include "ChargedParticle.hpp"

namespace KFCmd {
class PiPlusMeson : public ChargedParticle {
 public:
  explicit PiPlusMeson(const std::string&);
  virtual ~PiPlusMeson();
};
}  // namespace KFCmd

#endif
