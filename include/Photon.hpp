#ifndef __KFCMDPHOTON_HPP__
#define __KFCMDPHOTON_HPP__
#include <KFBase/Particle.hpp>

namespace KFCmd {
  class Photon : public KFBase::Particle {
  public:
    explicit Photon(const std::string&);
    virtual ~Photon();
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
    void setVertexX(const std::string&);
    void setVertexY(const std::string&);
    void setVertexZ(const std::string&);
  private:
    ccgo::CommonParams* _vertexX;
    ccgo::CommonParams* _vertexY;
    ccgo::CommonParams* _vertexZ;
  };
}

#endif
