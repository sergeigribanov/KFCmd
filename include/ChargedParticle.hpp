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
    virtual double calcVertexComponent(const Eigen::VectorXd&,
				       KFBase::VERTEX_COMPONENT) const override final;
    virtual Eigen::VectorXd calcDVertexComponent(const Eigen::VectorXd&,
						 KFBase::VERTEX_COMPONENT) const override final;
    virtual Eigen::MatrixXd calcD2VertexComponent(const Eigen::VectorXd&,
						  KFBase::VERTEX_COMPONENT) const override final;
  };
}

#endif
