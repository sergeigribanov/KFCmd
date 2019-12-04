#include <cmath>
#include "Photon.hpp"

KFCmd::Photon::Photon(const std::string& name):
  KFBase::Particle(name, 5) {
  setPeriod(2, 0, 2 * TMath::Pi());
}

KFCmd::Photon::~Photon(){
}

double KFCmd::Photon::calcMomentumComponent(const Eigen::VectorXd& x,
						 KFBase::MOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  double result = 0;
  // 0 --- energy
  // 1 --- R
  // 2 --- phi
  // 3 --- z0
  // 4 --- tan theta
  switch (component) {
  case KFBase::MOMENT_X:
    result = x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    break;
  case KFBase::MOMENT_Y:
    result = x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    break;
  case KFBase::MOMENT_Z:
    result = x(bi) * std::cos(x(bi + 4));
    break;
  case KFBase::MOMENT_E:
    result = x(bi);
    break;
  }
  return result;
}

Eigen::VectorXd KFCmd::Photon::calcDMomentumComponent
(const Eigen::VectorXd& x, KFBase::MOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
  case KFBase::MOMENT_X:
    // x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi) = std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi + 2) = -x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi + 4) = x(bi) * std::cos(x(bi + 4)) * std::cos(x(bi + 2));
    return result;
    break;
  case KFBase::MOMENT_Y:
    //  x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi) = std::sin(x(bi + 4)) * std::sin(x(bi + 2));
    result(bi + 2) = x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
    result(bi + 4) = x(bi) * std::cos(x(bi + 4)) * std::sin(x(bi + 2));
    break;
  case KFBase::MOMENT_Z:
    // x(bi) * std::cos(x(bi + 4));
    result(bi) = std::cos(x(bi + 4));
    result(bi + 4) = -x(bi) * std::sin(x(bi + 4));
    break;
  case KFBase::MOMENT_E:
    result(bi) = 1;
    break;
  }
  return result;
}

Eigen::MatrixXd KFCmd::Photon::calcD2MomentumComponent
(const Eigen::VectorXd& x, KFBase::MOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  switch (component) {
  case KFBase::MOMENT_X:
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
  case KFBase::MOMENT_Y:
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
  case KFBase::MOMENT_Z:
    // x(bi) * std::cos(x(bi + 4));
    result(bi, bi + 4) = -std::sin(x(bi + 4));
    result(bi + 4, bi) = result(bi, bi + 4);
    result(bi + 4, bi + 4) = -x(bi) * std::cos(x(bi + 4));
    break;
  case KFBase::MOMENT_E:
    break;
  }
  return result;
}

double KFCmd::Photon::calcVertexComponent
(const Eigen::VectorXd& x, KFBase::VERTEX_COMPONENT component) const {
  long bi = getBeginIndex();
  double result = 0;
  switch (component) {
  case KFBase::VERTEX_X:
    break;
  case KFBase::VERTEX_Y:
    break;
  case KFBase::VERTEX_Z:
    result = x(bi + 3) - x(bi + 1) / std::tan(x(bi + 4));
    break;
  }
  return result;
}

Eigen::VectorXd KFCmd::Photon::calcDVertexComponent
(const Eigen::VectorXd& x, KFBase::VERTEX_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
    case KFBase::VERTEX_X:
    break;
  case KFBase::VERTEX_Y:
    break;
  case KFBase::VERTEX_Z:
    result(bi + 1) = -1. / std::tan(x(bi + 4));
    result(bi + 3) = 1;
    double tmp = std::sin(x(bi + 4));
    result(bi + 4) = x(bi + 1) / tmp / tmp;
    break;
  }
  return result;
}

Eigen::MatrixXd KFCmd::Photon::calcD2VertexComponent
(const Eigen::VectorXd& x, KFBase::VERTEX_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  switch (component) {
  case KFBase::VERTEX_X:
    break;
  case KFBase::VERTEX_Y:
    break;
  case KFBase::VERTEX_Z:
    double tmp = std::sin(x(bi + 4));
    result(bi + 1, bi + 4) = 1. / tmp / tmp;
    result(bi + 4, bi + 1) = result(bi + 1, bi + 4);
    result(bi + 4, bi + 4) =
      -2 * x(bi + 1) * std::cos(x(bi + 4)) / tmp / tmp / tmp;
    break;
  }
  return result;
}
