#ifndef __KFCMDCHARGEDPARTICLE_HPP__
#define __KFCMDCHARGEDPARTICLE_HPP__
#include <KFBase/KFParticle.hpp>

namespace KFCmd {
  class KFCmdChargedParticle : public KFBase::KFParticle {
  public:
    KFCmdChargedParticle(const std::string&, double);
    virtual ~KFCmdChargedParticle();
    virtual double calcMomentumComponent(const Eigen::VectorXd&,
					 KFBase::KFMOMENT_COMPONENT) const override final;
    virtual Eigen::VectorXd calcDMomentumComponent(const Eigen::VectorXd&,
						   KFBase::KFMOMENT_COMPONENT) const override final;
    virtual Eigen::MatrixXd calcD2MomentumComponent(const Eigen::VectorXd&,
						    KFBase::KFMOMENT_COMPONENT) const override final;
  };
}

#endif
