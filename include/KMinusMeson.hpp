#ifndef __KFCMD_KMINUSMESON_HPP__
#define __KFCMD_KMINUSMESON_HPP__
#include "ChargedParticle.hpp"

namespace KFCmd {
  class KMinusMeson : public ChargedParticle {
  public:
    explicit KMinusMeson(const std::string&);
    virtual ~KMinusMeson();
  };
}

#endif
