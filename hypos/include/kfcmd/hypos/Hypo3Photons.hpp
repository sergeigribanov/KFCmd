#ifndef _KFCmd_Hypo3Photons_HPP_
#define _KFCmd_Hypo3Photons_HPP_

#include "kfcmd/core/Hypothesis.hpp"

namespace kfcmd {
  namespace hypos {

    class Hypo3Photons : public kfcmd::core::Hypothesis {
    public:
      Hypo3Photons(double, double=0., long = 20, double = 1.e-5);
      virtual ~Hypo3Photons();
    };

  }
}

#endif
