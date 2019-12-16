#ifndef __KFCMD_POSITRON_HPP__
#define __KFCMD_POSITRON_HPP__
#include "ChargedParticle.hpp"

namespace KFCmd {
class Positron : public ChargedParticle {
 public:
  explicit Positron(const std::string&);
  virtual ~Positron();
};
}  // namespace KFCmd

#endif
