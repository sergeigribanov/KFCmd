#include "ChargedParticle.hpp"

#include <TMath.h>

#include <cmath>

const double KFCmd::ChargedParticle::_c = 2.99792458;

KFCmd::ChargedParticle::ChargedParticle(const std::string& name, double mass,
                                        double charge)
    : KFBase::Particle(name, 5, mass, charge) {
  setPeriod(2, 0, 2 * TMath::Pi());
}

KFCmd::ChargedParticle::~ChargedParticle() {}

double KFCmd::ChargedParticle::calcMomentumComponent(
    const Eigen::VectorXd& x, KFBase::MOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  // 0 --- pt
  // 1 --- ctg theta
  // 2 --- phi
  // 3 --- R
  // 4 --- z
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

Eigen::VectorXd KFCmd::ChargedParticle::calcDMomentumComponent(
    const Eigen::VectorXd& x, KFBase::MOMENT_COMPONENT component) const {
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
      double en =
          std::pow(pt * pt * (1 + ctg * ctg) + getMass() * getMass(), 0.5);
      result(bi) = pt * (1 + ctg * ctg) / en;
      result(bi + 1) = ctg * pt * pt / en;
      break;
  }
  return result;
}

Eigen::MatrixXd KFCmd::ChargedParticle::calcD2MomentumComponent(
    const Eigen::VectorXd& x, KFBase::MOMENT_COMPONENT component) const {
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
      double en =
          std::pow(pt * pt * (1 + ctg * ctg) + getMass() * getMass(), 0.5);
      double tmp = 1 + ctg * ctg;
      result(bi, bi) = tmp / en - std::pow(pt * tmp, 2) / std::pow(en, 3);
      result(bi, bi + 1) =
          2 * pt * ctg / en - std::pow(pt, 3) * ctg * tmp / std::pow(en, 3);
      result(bi + 1, bi) = result(bi, bi + 1);
      result(bi + 1, bi + 1) =
          pt * pt / en - std::pow(ctg * pt * pt, 2) / std::pow(en, 3);
      break;
  }
  return result;
}

double KFCmd::ChargedParticle::calcVertexComponent(
    const Eigen::VectorXd& x, KFBase::VERTEX_COMPONENT component) const {
  long bi = getBeginIndex();
  double result = 0;
  double ct = x(_timeParam->getBeginIndex());
  double m = getMass();
  double m2 = m * m;
  double eta = x(bi + 1);
  double eta2 = eta * eta;
  double denominator = std::sqrt(m2 + (1 + eta2) * x(bi) * x(bi));
  double qBc = getCharge() * getMagnetField() * _c;
  double r = x(bi) / qBc;
  double w = qBc / denominator;
  switch (component) {
    case KFBase::VERTEX_X:
      result = -(x(bi + 3) + r) * std::sin(x(bi + 2)) +
               r * std::sin(w * ct + x(bi + 2));
      break;
    case KFBase::VERTEX_Y:
      result = (x(bi + 3) + r) * std::cos(x(bi + 2)) -
               r * std::cos(w * ct + x(bi + 2));
      break;
    case KFBase::VERTEX_Z:
      result = x(bi + 4) + x(bi) * eta * ct / denominator;
      break;
  }

  return result;
}

Eigen::VectorXd KFCmd::ChargedParticle::calcDVertexComponent(
    const Eigen::VectorXd& x, KFBase::VERTEX_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  double ct = x(_timeParam->getBeginIndex());
  double m = getMass();
  double m2 = m * m;
  double eta = x(bi + 1);
  double eta2 = eta * eta;
  double denominator = std::sqrt(m2 + (1 + eta2) * x(bi) * x(bi));
  double denom3 = std::pow(denominator, 3);
  double pt2 = x(bi) * x(bi);
  double qBc = getCharge() * getMagnetField() * _c;
  double r = x(bi) / qBc;
  double w = qBc / denominator;
  double sinP = std::sin(x(bi + 2));
  double cosP = std::cos(x(bi + 2));
  double alpha = w * ct + x(bi + 2);
  double sinA = std::sin(alpha);
  double cosA = std::cos(alpha);
  switch (component) {
    case KFBase::VERTEX_X:
      result(bi) =
          -sinP / qBc + sinA / qBc - pt2 * (1 + eta2) * ct * cosA / denom3;
      result(bi + 1) = -eta * pt2 * x(bi) * ct * cosA / denom3;
      result(bi + 2) = -(x(bi + 3) + r) * std::cos(x(bi + 2)) + r * cosA;
      result(bi + 3) = -sinP;
      result(_timeParam->getBeginIndex()) = r * w * cosA;
      break;
    case KFBase::VERTEX_Y:
      result(bi) =
          cosP / qBc - cosA / qBc - pt2 * (1 + eta2) * ct * sinA / denom3;
      result(bi + 1) = -eta * pt2 * x(bi) * ct * sinA / denom3;
      result(bi + 2) = -(x(bi + 3) + r) * std::sin(x(bi + 2)) + r * sinA;
      result(bi + 3) = cosP;
      result(_timeParam->getBeginIndex()) = r * w * sinA;
      break;
    case KFBase::VERTEX_Z:
      result(bi) =
          eta * ct / denominator - pt2 * eta * (1 + eta2) * ct / denom3;
      result(bi + 1) =
          x(bi) * ct / denominator - eta2 * pt2 * x(bi) * ct / denom3;
      result(bi + 4) = 1;
      result(_timeParam->getBeginIndex()) = x(bi) * eta / denominator;
      break;
  }
  return result;
}

Eigen::MatrixXd KFCmd::ChargedParticle::calcD2VertexComponent(
    const Eigen::VectorXd& x, KFBase::VERTEX_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  double ct = x(_timeParam->getBeginIndex());
  double m = getMass();
  double m2 = m * m;
  double eta = x(bi + 1);
  double eta2 = eta * eta;
  double denominator = std::sqrt(m2 + (1 + eta2) * x(bi) * x(bi));
  double denom3 = std::pow(denominator, 3);
  double denom5 = std::pow(denominator, 5);
  double pt2 = x(bi) * x(bi);
  double pt3 = pt2 * x(bi);
  double a1 = 1 + eta2;
  double a2 = a1 * a1;
  double ct2 = ct * ct;
  double qBc = getCharge() * getMagnetField() * _c;
  double r = x(bi) / qBc;
  double w = qBc / denominator;
  double sinP = std::sin(x(bi + 2));
  double cosP = std::cos(x(bi + 2));
  double alpha = w * ct + x(bi + 2);
  double sinA = std::sin(alpha);
  double cosA = std::cos(alpha);
  double denom6 = std::pow(denominator, 6);
  double pt4 = pt2 * pt2;
  double pt5 = pt4 * x(bi);
  switch (component) {
    case KFBase::VERTEX_X:
      result(bi, bi) = -3. * x(bi) * a1 * ct * cosA / denom3 +
                       3. * pt3 * a2 * ct * cosA / denom5 -
                       pt3 * a2 * ct2 * qBc * sinA / denom6;
      result(bi, bi + 1) = -3. * pt2 * eta * ct * cosA / denom3 +
                           3. * pt4 * a1 * eta * ct * cosA / denom5 -
                           pt4 * a1 * eta * ct2 * qBc * sinA / denom6;
      result(bi + 1, bi) = result(bi, bi + 1);
      result(bi, bi + 2) =
          -cosP / qBc + cosA / qBc + pt2 * a1 * ct * sinA / denom3;
      result(bi + 2, bi) = result(bi, bi + 2);
      result(bi, _timeParam->getBeginIndex()) =
          cosA / denominator - pt2 * a1 * cosA / denom3 +
          pt2 * a1 * w * ct * sinA / denom3;
      result(_timeParam->getBeginIndex(), bi) =
          result(bi, _timeParam->getBeginIndex());
      result(bi + 1, bi + 1) = -pt3 * ct * cosA / denom3 +
                               3. * pt5 * eta2 * ct * cosA / denom5 -
                               pt5 * eta2 * ct2 * qBc * sinA / denom6;
      result(bi + 1, bi + 2) = pt3 * eta * ct * sinA / denom3;
      result(bi + 2, bi + 1) = result(bi + 1, bi + 2);
      result(bi + 1, _timeParam->getBeginIndex()) =
          -pt3 * eta * cosA / denom3 + pt3 * eta * w * ct * sinA / denom3;
      result(_timeParam->getBeginIndex(), bi + 1) =
          result(bi + 1, _timeParam->getBeginIndex());
      result(bi + 2, bi + 2) = (x(bi + 3) + r) * sinP - r * sinA;
      result(bi + 2, bi + 3) = -cosP;
      result(bi + 3, bi + 2) = result(bi + 2, bi + 3);
      result(bi + 2, _timeParam->getBeginIndex()) = -r * w * sinA;
      result(_timeParam->getBeginIndex(), bi + 2) =
          result(bi + 2, _timeParam->getBeginIndex());
      result(_timeParam->getBeginIndex(), _timeParam->getBeginIndex()) =
          -r * w * w * sinA;
      break;
    case KFBase::VERTEX_Y:
      result(bi, bi) = -3. * x(bi) * a1 * ct * sinA / denom3 +
                       3. * pt3 * a2 * ct * sinA / denom5 +
                       qBc * pt3 * a2 * ct2 * cosA / denom6;
      result(bi, bi + 1) = -3. * pt2 * eta * ct * sinA / denom3 +
                           3. * pt4 * eta * a1 * ct * sinA / denom5 +
                           pt4 * eta * (1 + eta2) * ct2 * qBc * cosA / denom6;
      result(bi + 1, bi) = result(bi, bi + 1);
      result(bi, bi + 2) =
          -sinP / qBc + sinA / qBc - pt2 * a1 * ct * cosA / denom3;
      result(bi + 2, bi) = result(bi, bi + 2);
      result(bi, _timeParam->getBeginIndex()) =
          sinA * w / qBc - pt2 * a1 * sinA / denom3 -
          pt2 * a1 * w * ct * cosA / denom3;
      result(_timeParam->getBeginIndex(), bi) =
          result(bi, _timeParam->getBeginIndex());
      result(bi + 1, bi + 1) = -pt3 * ct * sinA / denom3 +
                               3. * eta2 * pt5 * ct * sinA / denom5 +
                               eta2 * ct2 * qBc * pt5 * cosA / denom6;
      result(bi + 1, bi + 2) = -pt3 * eta * ct * cosA / denom3;
      result(bi + 2, bi + 1) = result(bi + 1, bi + 2);
      result(bi + 1, _timeParam->getBeginIndex()) =
          -pt3 * eta * sinA / denom3 - pt3 * eta * w * ct * cosA / denom3;
      result(_timeParam->getBeginIndex(), bi + 1) =
          result(bi + 1, _timeParam->getBeginIndex());
      result(bi + 2, bi + 2) = -(x(bi + 3) + r) * cosP + r * cosA;
      result(bi + 2, bi + 3) = -sinP;
      result(bi + 3, bi + 2) = result(bi + 2, bi + 3);
      result(bi + 2, _timeParam->getBeginIndex()) = r * w * cosA;
      result(_timeParam->getBeginIndex(), bi + 2) =
          result(bi + 2, _timeParam->getBeginIndex());
      result(_timeParam->getBeginIndex(), _timeParam->getBeginIndex()) =
          r * w * w * cosA;
      break;
    case KFBase::VERTEX_Z:
      result(bi, bi) = -3. * x(bi) * eta * a1 * ct / denom3 +
                       3. * pt3 * a2 * eta * ct / denom5;
      result(bi, bi + 1) = ct / denominator -
                           pt2 * (1 + 4 * eta2) * ct / denom3 +
                           3. * eta2 * a1 * pt4 * ct / denom5;
      result(bi + 1, bi) = result(bi, bi + 1);
      result(bi, _timeParam->getBeginIndex()) =
          eta / denominator - pt2 * eta * a1 / denom3;
      result(_timeParam->getBeginIndex(), bi) =
          result(bi, _timeParam->getBeginIndex());
      result(bi + 1, bi + 1) =
          -3. * eta * pt3 * ct / denom3 + 3. * eta * eta2 * pt5 * ct / denom5;
      result(bi + 1, _timeParam->getBeginIndex()) =
          x(bi) / denominator - eta2 * pt3 / denom3;
      result(_timeParam->getBeginIndex(), bi + 1) =
          result(bi + 1, _timeParam->getBeginIndex());
      break;
  }
  return result;
}

void KFCmd::ChargedParticle::setMagnetField(const std::string& name) {
  const auto it = getConstants()->find(name);
  if (it == getConstants()->end()) {
    // TO DO : exception
  }
  _magnetFieldIterator = it;
}

void KFCmd::ChargedParticle::setTimeParameter(const std::string& name) {
  auto it = getCommonParameters()->find(name);
  if (it == getCommonParameters()->end()) {
    // TO DO : exception
  }
  _timeParam = it->second;
}

double KFCmd::ChargedParticle::getMagnetField() const {
  return _magnetFieldIterator->second;
}
