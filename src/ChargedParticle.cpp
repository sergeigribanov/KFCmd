#include <cmath>
#include <TMath.h>
#include "ChargedParticle.hpp"

KFCmd::ChargedParticle::ChargedParticle(const std::string& name,
					double mass, double charge):
  KFBase::Particle(name, 5, mass, charge) {
  setPeriod(2, 0, 2 * TMath::Pi());
}

KFCmd::ChargedParticle::~ChargedParticle() {
}

double KFCmd::ChargedParticle::calcMomentumComponent(const Eigen::VectorXd& x,
							  KFBase::MOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  // 0 --- pt
  // 1 --- ctg theta
  // 2 --- phi
  double result = 0;
  switch (component) {
  case KFBase::MOMENT_X:
    result = x(bi) * std::cos(x(bi + 2));
    break;
  case KFBase::MOMENT_Y:
    result = x(bi) * std::sin(x(bi + 2));
    break;
  case KFBase::MOMENT_Z:
    result = x(bi) * x(bi + 1);
    break;
  case KFBase::MOMENT_E:
    double pt = x(bi);
    double ctg = x(bi + 1);
    result = std::pow(pt * pt * (1 + ctg * ctg) + getMass() * getMass(), 0.5);
    break;
  }
  return result;
}

Eigen::VectorXd KFCmd::ChargedParticle::calcDMomentumComponent
(const Eigen::VectorXd& x, KFBase::MOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
  case KFBase::MOMENT_X:
    result(bi) = std::cos(x(bi + 2));
    result(bi + 2) = -x(bi) * std::sin(x(bi + 2));
    break;
  case KFBase::MOMENT_Y:
    result(bi) = std::sin(x(bi + 2));
    result(bi + 2) = x(bi) * std::cos(x(bi + 2));
    break;
  case KFBase::MOMENT_Z:
    result(bi) = x(bi + 1);
    result(bi + 1) = x(bi);
    break;
  case KFBase::MOMENT_E:
    double pt = x(bi);
    double ctg = x(bi + 1);
    double en =  std::pow(pt * pt * (1 + ctg * ctg) + getMass() * getMass(), 0.5);
    result(bi) = pt * (1 + ctg * ctg) / en;
    result(bi + 1) = ctg * pt * pt / en;
    break;
  }
  return result;
}

Eigen::MatrixXd KFCmd::ChargedParticle::calcD2MomentumComponent
(const Eigen::VectorXd& x, KFBase::MOMENT_COMPONENT component) const { 
  long bi = getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  switch (component) {
  case KFBase::MOMENT_X:
    result(bi, bi + 2) = -std::sin(x(bi + 2));
    result(bi + 2, bi) = result(bi, bi + 2);
    result(bi + 2, bi + 2) = -x(bi) * std::cos(x(bi + 2));
    break;
  case KFBase::MOMENT_Y:
    result(bi, bi + 2) = std::cos(x(bi + 2));
    result(bi + 2, bi) = result(bi, bi + 2);
    result(bi + 2, bi + 2) = -x(bi) * std::sin(x(bi + 2));
    break;
  case KFBase::MOMENT_Z:
    result(bi, bi + 1) = 1;
    result(bi + 1, bi) = 1;
    break;
  case KFBase::MOMENT_E:
    double pt = x(bi);
    double ctg = x(bi + 1);
    double en =  std::pow(pt * pt * (1 + ctg * ctg) + getMass() * getMass(), 0.5);
    double tmp = 1 + ctg * ctg;
    result(bi, bi) = tmp / en - std::pow(pt * tmp, 2) / std::pow(en, 3);
    result(bi, bi + 1) = 2 * pt * ctg / en - std::pow(pt, 3) * ctg * tmp / std::pow(en, 3);
    result(bi + 1, bi) = result(bi, bi + 1);
    result(bi + 1, bi + 1) = pt * pt / en - std::pow(ctg * pt * pt, 2) / std::pow(en, 3);
    break;
  }
  return result;
}

double KFCmd::ChargedParticle::calcVertexComponent
(const Eigen::VectorXd& x, KFBase::VERTEX_COMPONENT component) const {
  long bi = getBeginIndex();
  double result = 0;
  switch (component) {
  case KFBase::VERTEX_X:
    result = x(bi + 3) * std::sin(x(bi + 2));
    break;
  case KFBase::VERTEX_Y:
    result = x(bi + 3) * std::cos(x(bi + 2));
    break;
  case KFBase::VERTEX_Z:
    result = x(bi + 4);
    break;
  }
  return result;
}

Eigen::VectorXd KFCmd::ChargedParticle::calcDVertexComponent
(const Eigen::VectorXd& x, KFBase::VERTEX_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
  case KFBase::VERTEX_X:
    result(bi + 2) = x(bi + 3) * std::cos(x(bi + 2));
    result(bi + 3) = std::sin(x(bi + 2));
    break;
  case KFBase::VERTEX_Y:
    result(bi + 2) = -x(bi + 3) * std::sin(x(bi + 2));
    result(bi + 3) = std::cos(x(bi + 2));
    break;
  case KFBase::VERTEX_Z:
    result(bi + 4) = 1;
    break;
  }
  return result;
}

Eigen::MatrixXd KFCmd::ChargedParticle::calcD2VertexComponent
(const Eigen::VectorXd& x, KFBase::VERTEX_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  switch (component) {
  case KFBase::VERTEX_X:
    result(bi + 2, bi + 2) = -x(bi + 3) * std::sin(x(bi + 2));
    result(bi + 2, bi + 3) = std::cos(x(bi + 2));
    result(bi + 3, bi + 2) = result(bi + 2, bi + 3);
    break;
  case KFBase::VERTEX_Y:
    result(bi + 2, bi + 2) = -x(bi + 3) * std::cos(x(bi + 2));
    result(bi + 2, bi + 3) = -std::sin(x(bi + 2));
    result(bi + 3, bi + 2) = result(bi + 2, bi + 3);
    break;
  case KFBase::VERTEX_Z:
    break;
  }
  return result;
}
