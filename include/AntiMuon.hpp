#ifndef __KFCMD_ANTIMUON_HPP__
#define __KFCMD_ANTIMUON_HPP__
#include "ChargedParticle.hpp"

namespace KFCmd {
class AntiMuon : public ChargedParticle {
 public:
  explicit AntiMuon(const std::string&);
  virtual ~AntiMuon();
};
}  // namespace KFCmd

#endif
