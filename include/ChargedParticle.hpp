#ifndef __KFCMD_CHARGEDPARTICLE_HPP__
#define __KFCMD_CHARGEDPARTICLE_HPP__
#include <KFBase/Particle.hpp>

namespace KFCmd {
  class ChargedParticle : public KFBase::Particle {
  public:
    ChargedParticle(const std::string&, double);
    virtual ~ChargedParticle();
    virtual double calcMomentumComponent(const Eigen::VectorXd&,
					 KFBase::MOMENT_COMPONENT) const override final;
    virtual Eigen::VectorXd calcDMomentumComponent(const Eigen::VectorXd&,
						   KFBase::MOMENT_COMPONENT) const override final;
    virtual Eigen::MatrixXd calcD2MomentumComponent(const Eigen::VectorXd&,
						    KFBase::MOMENT_COMPONENT) const override final;
  };
}

#endif
