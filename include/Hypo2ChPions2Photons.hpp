#ifndef __KFCMD_HYPO2CHPIONS2PHOTONS_HPP__
#define __KFCMD_HYPO2CHPIONS2PHOTONS_HPP__
#include "Hypothesis.hpp"

namespace KFCmd {
  class Hypo2ChPions2Photons : public Hypothesis {
  public:
    Hypo2ChPions2Photons(double, double, long = 20, double = 1.e-3);
    virtual ~Hypo2ChPions2Photons();
  };
}

#endif
