#ifndef __KFCMD_HYPO2PHOTONS_HPP__
#define __KFCMD_HYPO2PHOTONS_HPP__
#include <KFBase/Hypothesis.hpp>

namespace KFCmd {
  class Hypo2Photons : public KFBase::Hypothesis {
  public:
    explicit Hypo2Photons(double);
    virtual ~Hypo2Photons();
  };
}

#endif
