#ifndef __KFCMD_ELECTRON_HPP__
#define __KFCMD_ELECTRON_HPP__
#include "ChargedParticle.hpp"

namespace KFCmd {
  class Electron : public ChargedParticle {
  public:
    explicit Electron(const std::string&);
    virtual ~Electron();
  };
}

#endif
