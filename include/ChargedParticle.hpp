#ifndef __KFCMD_CHARGEDPARTICLE_HPP__
#define __KFCMD_CHARGEDPARTICLE_HPP__
#include <KFBase/VertexParticle.hpp>
#include <ccgo/CommonParams.hpp>

namespace KFCmd {
class ChargedParticle : public KFBase::VertexParticle {
 public:
  ChargedParticle(const std::string&, double, double);
  virtual ~ChargedParticle();
  void setMagnetField(const std::string&);
  void setTimeParameter(const std::string&);
  virtual double calcMomentumComponent(
      const Eigen::VectorXd&, KFBase::MOMENT_COMPONENT) const override final;
  virtual Eigen::VectorXd calcDMomentumComponent(
      const Eigen::VectorXd&, KFBase::MOMENT_COMPONENT) const override final;
  virtual Eigen::MatrixXd calcD2MomentumComponent(
      const Eigen::VectorXd&, KFBase::MOMENT_COMPONENT) const override final;
  virtual double calcVertexComponent(
      const Eigen::VectorXd&, KFBase::VERTEX_COMPONENT) const override final;
  virtual Eigen::VectorXd calcDVertexComponent(
      const Eigen::VectorXd&, KFBase::VERTEX_COMPONENT) const override final;
  virtual Eigen::MatrixXd calcD2VertexComponent(
      const Eigen::VectorXd&, KFBase::VERTEX_COMPONENT) const override final;

 protected:
  double getMagnetField() const;
  ccgo::CommonParams* _timeParam;

 private:
  static const double _c;
  std::unordered_map<std::string, double>::const_iterator _magnetFieldIterator;
};
}  // namespace KFCmd

#endif
