#ifndef __KFCMD_HYPO2CHPIONS_HPP__
#define __KFCMD_HYPO2CHPIONS_HPP__
#include <KFBase/Hypothesis.hpp>

namespace KFCmd {
  class Hypo2ChPions : public KFBase::Hypothesis {
  public:
    explicit Hypo2ChPions(double);
    virtual ~Hypo2ChPions();
  };
}

#endif
