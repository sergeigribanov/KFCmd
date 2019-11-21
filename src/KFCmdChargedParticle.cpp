#include <iostream>
#include <cmath>
#include "KFCmdChargedParticle.hpp"

KFCmdChargedParticle::KFCmdChargedParticle(const std::string& name, double mass):
  KFBase::KFParticle(name, 5, mass) {
}

KFCmdChargedParticle::~KFCmdChargedParticle() {
}

double KFCmdChargedParticle::calcMomentumComponent(const Eigen::VectorXd& x,
						   KFBase::KFMOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  // 0 --- pt
  // 1 --- ctg theta
  // 2 --- phi
  double result;
  switch (component) {
  case KFBase::KFMOMENT_X:
    result = x(bi) * std::cos(x(bi + 2));
    break;
  case KFBase::KFMOMENT_Y:
    result = x(bi) * std::sin(x(bi + 2));
    break;
  case KFBase::KFMOMENT_Z:
    result = x(bi) * x(bi + 1);
    break;
  case KFBase::KFMOMENT_E:
    double pt = x(bi);
    double ctg = x(bi + 1);
    result = std::pow(pt * pt * (1 + ctg * ctg) + getMass() * getMass(), 0.5);
    break;
  }
  return result;
}

Eigen::VectorXd KFCmdChargedParticle::calcDMomentumComponent
(const Eigen::VectorXd& x, KFBase::KFMOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
    case KFBase::KFMOMENT_X:
      result(bi) = std::cos(x(bi + 2));
      result(bi + 2) = -x(bi) * std::sin(x(bi + 2));
      break;
  case KFBase::KFMOMENT_Y:
    result(bi) = std::sin(x(bi + 2));
    result(bi + 2) = x(bi) * std::cos(x(bi + 2));
    break;
  case KFBase::KFMOMENT_Z:
    result(bi) = x(bi + 1);
    result(bi + 1) = x(bi);
    break;
  case KFBase::KFMOMENT_E:
    double pt = x(bi);
    double ctg = x(bi + 1);
    double en =  std::pow(pt * pt * (1 + ctg * ctg) + getMass() * getMass(), 0.5);
    result(bi) = pt * (1 + ctg * ctg) / en;
    result(bi + 1) = ctg * pt * pt / en;
    break;
  }
  return result;
}

Eigen::MatrixXd KFCmdChargedParticle::calcD2MomentumComponent
(const Eigen::VectorXd& x, KFBase::KFMOMENT_COMPONENT component) const { 
  long bi = getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  switch (component) {
  case KFBase::KFMOMENT_X:
    result(bi, bi + 2) = -std::sin(x(bi + 2));
    result(bi + 2, bi) = result(bi, bi + 2);
    result(bi + 2, bi + 2) = -x(bi) * std::cos(x(bi + 2));
    break;
  case KFBase::KFMOMENT_Y:
    result(bi, bi + 2) = std::cos(x(bi + 2));
    result(bi + 2, bi) = result(bi, bi + 2);
    result(bi + 2, bi + 2) = -x(bi) * std::sin(x(bi + 2));
    break;
  case KFBase::KFMOMENT_Z:
    result(bi, bi + 1) = 1;
    result(bi + 1, bi) = 1;
    break;
  case KFBase::KFMOMENT_E:
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
