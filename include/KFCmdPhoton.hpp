#ifndef __KFCMDPHOTON_HPP__
#define __KFCMDPHOTON_HPP__
#include <KFBase/KFParticle.hpp>

namespace KFCmd {
  class KFCmdPhoton : public KFBase::KFParticle {
  public:
    explicit KFCmdPhoton(const std::string&);
    virtual ~KFCmdPhoton();
    virtual double calcMomentumComponent(const Eigen::VectorXd&,
					 KFBase::KFMOMENT_COMPONENT) const override final;
    virtual Eigen::VectorXd calcDMomentumComponent(const Eigen::VectorXd&,
						   KFBase::KFMOMENT_COMPONENT) const override final;
    virtual Eigen::MatrixXd calcD2MomentumComponent(const Eigen::VectorXd&,
						    KFBase::KFMOMENT_COMPONENT) const override final;
  };
}

#endif
