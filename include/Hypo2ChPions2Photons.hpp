#ifndef __KFCMD_HYPO2CHPIONS2PHOTONS_HPP__
#define __KFCMD_HYPO2CHPIONS2PHOTONS_HPP__
#include <KFBase/Hypothesis.hpp>

namespace KFCmd {
  class Hypo2ChPions2Photons : public KFBase::Hypothesis {
  public:
    explicit Hypo2ChPions2Photons(double);
    virtual ~Hypo2ChPions2Photons();
  };
}

#endif
