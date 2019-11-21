#ifndef __KFCMD_CHARGEDPIMESON_HPP__
#define __KFCMD_CHARGEDPIMESON_HPP__
#include "ChargedParticle.hpp"

namespace KFCmd {
  class ChargedPiMeson : public ChargedParticle {
  public:
    explicit ChargedPiMeson(const std::string&);
    virtual ~ChargedPiMeson();
  };
}

#endif
