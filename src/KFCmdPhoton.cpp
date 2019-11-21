#include <cmath>
#include "KFCmdPhoton.hpp"

KFCmdPhoton::KFCmdPhoton(const std::string& name):
  KFBase::KFParticle(name, 5) {
}

KFCmdPhoton::~KFCmdPhoton(){
}

double KFCmdPhoton::calcMomentumComponent(const Eigen::VectorXd& x,
					  KFBase::KFMOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  double result;
  // 0 --- energy
  // 1 --- R
  // 2 --- phi
  // 3 --- z0
  // 4 --- tan theta
  switch (component) {
  case KFBase::KFMOMENT_X:
    result = x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    break;
  case KFBase::KFMOMENT_Y:
    result = x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    break;
  case KFBase::KFMOMENT_Z:
    result = x(bi) * std::cos(x(bi + 4));
    break;
  case KFBase::KFMOMENT_E:
    result = x(bi);
    break;
  }
  return result;
}

Eigen::VectorXd KFCmdPhoton::calcDMomentumComponent
(const Eigen::VectorXd& x, KFBase::KFMOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
  case KFBase::KFMOMENT_X:
    // x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi) = std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi + 2) = -x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi + 4) = x(bi) * std::cos(x(bi + 4)) * std::cos(x(bi + 2));
    return result;
    break;
  case KFBase::KFMOMENT_Y:
    //  x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi) = std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi + 2) = x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi + 4) = x(bi) * std::cos(x(bi + 4)) * std::sin(x(bi + 2));
    break;
  case KFBase::KFMOMENT_Z:
    // x(bi) * std::cos(x(bi + 4));
    result(bi) = std::cos(x(bi + 4));
    result(bi + 4) = -x(bi) * std::sin(x(bi + 4));
    break;
  case KFBase::KFMOMENT_E:
    result(bi) = 1;
    break;
  }
  return result;
}

Eigen::MatrixXd KFCmdPhoton::calcD2MomentumComponent
(const Eigen::VectorXd& x, KFBase::KFMOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  switch (component) {
  case KFBase::KFMOMENT_X:
    // x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi, bi + 2) = -std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi + 2, bi) = result(bi, bi + 2);
    result(bi, bi + 4) = std::cos(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi + 4, bi) = result(bi, bi + 4);
    result(bi + 2, bi + 2) = -x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi + 2, bi + 4) = -x(bi) * std::cos(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi + 4, bi + 2) = result(bi + 2, bi + 4);
    result(bi + 4, bi + 4) = result(bi + 2, bi + 2);
    break;
  case KFBase::KFMOMENT_Y:
    //  x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi, bi + 2) = std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi + 2, bi) = result(bi, bi + 2);
    result(bi, bi + 4) = std::cos(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi + 4, bi) = result(bi, bi + 4);
    result(bi + 2, bi + 2) = -x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi + 2, bi + 4) = x(bi) * std::cos(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi + 4, bi + 2) = result(bi + 2, bi + 4);
    result(bi + 4, bi + 4) = result(bi + 2, bi + 2);
    break;
  case KFBase::KFMOMENT_Z:
    // x(bi) * std::cos(x(bi + 4));
    result(bi, bi + 4) = -std::sin(x(bi + 4));
    result(bi + 4, bi) = result(bi, bi + 4);
    result(bi + 4, bi + 4) = -x(bi) * std::cos(x(bi + 4));
    break;
  }
  return result;
}
