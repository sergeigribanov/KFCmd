#ifndef _KFCmd_Hypo2ChPionsPhotonLostPhoton_HPP_
#define _KFCmd_Hypo2ChPionsPhotonLostPhoton_HPP_

#include <kfcmd/core/Hypothesis.hpp>

namespace kfcmd {
  namespace hypos {
    class Hypo2ChPionsPhotonLostPhoton : public kfcmd::core::Hypothesis {
    public:
      Hypo2ChPionsPhotonLostPhoton(double, double, long = 20, double = 1.e-4);
      virtual ~Hypo2ChPionsPhotonLostPhoton();
    };
  }
}

#endif
