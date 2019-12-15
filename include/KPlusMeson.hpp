#ifndef __KFCMD_KPLUSMESON_HPP__
#define __KFCMD_KPLUSMESON_HPP__
#include "ChargedParticle.hpp"

namespace KFCmd {
  class KPlusMeson : public ChargedParticle {
  public:
    explicit KPlusMeson(const std::string&);
    virtual ~KPlusMeson();
  };
}

#endif
