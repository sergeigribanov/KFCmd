#ifndef __KFCMD_PIMINUSMESON_HPP__
#define __KFCMD_PIMINUSMESON_HPP__
#include "ChargedParticle.hpp"

namespace KFCmd {
class PiMinusMeson : public ChargedParticle {
 public:
  explicit PiMinusMeson(const std::string&);
  virtual ~PiMinusMeson();
};
}  // namespace KFCmd

#endif
