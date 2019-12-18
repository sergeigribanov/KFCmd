#ifndef __KFCMD_MUON_HPP__
#define __KFCMD_MUON_HPP__
#include "ChargedParticle.hpp"

namespace KFCmd {
class Muon : public ChargedParticle {
 public:
  explicit Muon(const std::string&);
  virtual ~Muon();
};
}  // namespace KFCmd

#endif
