/*
 * KFCmd library
 * See LICENSE file at the top of the source tree.
 *
 * This product includes software developed by the
 * CMD-3 collaboration (https://cmd.inp.nsk.su/).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

/**
 * @file ChargedParticle.cpp
 *
 * @brief Implementation of ChargedParticle methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/core/ChargedParticle.hpp"

#include <TMath.h>
#include <cmath>
#include <iostream>

const double kfcmd::core::ChargedParticle::_c = 2.99792458;

kfcmd::core::ChargedParticle::ChargedParticle(const std::string& name, double mass,
                                        double charge)
    : kfbase::core::VertexParticle(name, 5, mass, charge) {
  setPeriod(2, 0, 2 * TMath::Pi());
  setLowerLimit(0, 0);
  setUpperLimit(0, 1100);
  setLowerLimit(1, -20);
  setUpperLimit(1, 20);
  setLowerLimit(2, -1000 * TMath::Pi());
  setUpperLimit(2, 1000 * TMath::Pi());
  setLowerLimit(3, -30);
  setUpperLimit(3, 30);
  setLowerLimit(4, -20);
  setUpperLimit(4, 20);
}

kfcmd::core::ChargedParticle::~ChargedParticle() {}

double kfcmd::core::ChargedParticle::calcMomentumComponent(
    const Eigen::VectorXd& x, kfbase::core::MOMENT_COMPONENT component) const {
  // 0 --- pt
  // 1 --- ctg theta
  // 2 --- phi
  // 3 --- rho
  // 4 --- tz
  const long pt_i = getBeginIndex();
  double result = 0;
  double ct = x(_timeParam->getBeginIndex());
  double pt = x(pt_i);
  double eta = x(pt_i + 1);
  double m = getMass();
  double energy = std::sqrt(m * m + (1 + eta * eta) * pt * pt);
  double qBc = getCharge() * getMagneticField() * _c;
  double w = qBc / energy;
  double alpha = w * ct - x(pt_i + 2);
  switch (component) {
    case kfbase::core::MOMENT_X:
      result = pt * std::cos(alpha);
      break;
    case kfbase::core::MOMENT_Y:
      result = -pt * std::sin(alpha);
      break;
    case kfbase::core::MOMENT_Z:
      result = pt * eta;
      break;
    case kfbase::core::MOMENT_E:
      result = energy;
      break;
  }
  return result;
}

Eigen::VectorXd kfcmd::core::ChargedParticle::calcDMomentumComponent(
    const Eigen::VectorXd& x, kfbase::core::MOMENT_COMPONENT component) const {
  // 0 --- pt
  // 1 --- ctg theta
  // 2 --- phi
  // 3 --- rho
  // 4 --- tz
  const long pt_i = getBeginIndex();
  const long time_i = _timeParam->getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  double ct = x(time_i);
  double m = getMass();
  double m2 = m * m;
  double eta = x(pt_i + 1);
  double eta2 = eta * eta;
  double pt = x(pt_i);
  double pt2 = pt * pt;
  double energy = std::sqrt(m2 + (1 + eta2) * pt2);
  double qBc = getCharge() * getMagneticField() * _c;
  double w = qBc / energy;
  double dw_de = -w / energy;
  double de_dpt = pt * (1 + eta2) / energy;
  double de_deta = pt2 * eta / energy;
  double dw_dpt = dw_de * de_dpt;
  double dw_deta = dw_de * de_deta;
  double alpha = w * ct - x(pt_i + 2);
  double sinA = std::sin(alpha);
  double cosA = std::cos(alpha);
  double px = pt * cosA;
  double py = -pt * sinA;
  switch (component) {
    case kfbase::core::MOMENT_X:
      result(pt_i + 2) = -py;
      result(pt_i) = cosA + dw_dpt * py * ct;
      result(pt_i + 1) = dw_deta * py * ct;
      result(time_i) = w * py;
      break;
    case kfbase::core::MOMENT_Y:
      result(pt_i + 2) = px;
      result(pt_i) = -sinA - dw_dpt * px * ct;
      result(pt_i + 1) = -dw_deta * px * ct;
      result(time_i) = -w * px;
      break;
    case kfbase::core::MOMENT_Z:
      result(pt_i) = eta;
      result(pt_i + 1) = pt;
      break;
    case kfbase::core::MOMENT_E:
      result(pt_i) = de_dpt;
      result(pt_i + 1) = de_deta;
      break;
  }
  return result;
}

Eigen::MatrixXd kfcmd::core::ChargedParticle::calcD2MomentumComponent(
    const Eigen::VectorXd& x, kfbase::core::MOMENT_COMPONENT component) const {
  // 0 --- pt
  // 1 --- ctg theta
  // 2 --- phi
  // 3 --- rho
  // 4 --- tz
  const long pt_i = getBeginIndex();
  const long eta_i = pt_i + 1;
  const long phi_i = pt_i + 2;
  const long time_i = _timeParam->getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  double ct = x(time_i);
  double m = getMass();
  double m2 = m * m;
  double eta = x(eta_i);
  double eta2 = eta * eta;
  double pt = x(pt_i);
  double pt2 = pt * pt;
  double c0 = 1 + eta2;
  double energy = std::sqrt(m2 + c0 * pt2);
  double qBc = getCharge() * getMagneticField() * _c;
  double w = qBc / energy;
  double dw_de = -w / energy;
  double d2w_d2e = 2. * w / energy / energy;
  double de_dpt = pt * c0 / energy;
  double de_deta = pt2 * eta / energy;
  double d2e_d2pt = c0 / energy - de_dpt * de_dpt / energy;
  double d2e_dpt_deta = 2. * eta * pt / energy - de_dpt * de_deta / energy;
  double d2e_d2eta = pt2 / energy - de_deta * de_deta / energy;
  double dw_dpt = dw_de * de_dpt;
  double dw_deta = dw_de * de_deta;
  double d2w_d2pt = d2w_d2e * de_dpt * de_dpt + dw_de * d2e_d2pt;
  double d2w_dpt_deta = d2w_d2e * de_dpt * de_deta + dw_de * d2e_dpt_deta;
  double d2w_d2eta = d2w_d2e * de_deta * de_deta + dw_de * d2e_d2eta;
  double alpha = w * ct - x(phi_i);
  double sinA = std::sin(alpha);
  double cosA = std::cos(alpha);
  double px = pt * cosA;
  double py = -pt * sinA;
  double dpy_dpt = -sinA - dw_dpt * px * ct;
  double dpy_deta = -dw_deta * px * ct;
  double dpy_dct = -w * px;
  double dpx_dpt = cosA + dw_dpt * py * ct;
  double dpx_deta = dw_deta * py * ct;
  double dpx_dct = w * py;
  switch (component) {
    case kfbase::core::MOMENT_X:
      result(phi_i, phi_i) = -px;
      result(phi_i, pt_i) = -dpy_dpt;
      result(pt_i, phi_i) = result(phi_i, pt_i);
      result(phi_i, eta_i) = -dpy_deta;
      result(eta_i, phi_i) = result(phi_i, eta_i);
      result(phi_i, time_i) = -dpy_dct;
      result(time_i, phi_i) = result(phi_i, time_i);

      result(pt_i, pt_i) = -dw_dpt * ct * sinA +
       	d2w_d2pt * py * ct + dw_dpt * dpy_dpt * ct;
      result(pt_i, eta_i) = -dw_deta * ct * sinA +
	d2w_dpt_deta * py * ct + dw_dpt * dpy_deta * ct;
      result(eta_i, pt_i) = result(pt_i, eta_i);
      result(pt_i, time_i) = -w * sinA + dw_dpt * py + dw_dpt * dpy_dct * ct;
      result(time_i, pt_i) = result(pt_i, time_i);

      result(eta_i, eta_i) = d2w_d2eta * py * ct + dw_deta * dpy_deta * ct;
      result(eta_i, time_i) = dw_deta * py + dw_deta * dpy_dct * ct;
      result(time_i, eta_i) = result(eta_i, time_i);
      result(time_i, time_i) = w * dpy_dct;
     break;
    case kfbase::core::MOMENT_Y:
      result(phi_i, phi_i) = -py;
      result(phi_i, pt_i) = dpx_dpt;
      result(pt_i, phi_i) = result(phi_i, pt_i);
      result(phi_i, eta_i) = dpx_deta;
      result(eta_i, phi_i) = result(phi_i, eta_i);
      result(phi_i, time_i) = dpx_dct;
      result(time_i, phi_i) = result(phi_i, time_i);

      result(pt_i, pt_i) = -dw_dpt * ct * cosA -
       	d2w_d2pt * px * ct - dw_dpt * dpx_dpt * ct;
      result(pt_i, eta_i) = -dw_deta * ct * cosA -
	d2w_dpt_deta * px * ct - dw_dpt * dpx_deta * ct;
      result(eta_i, pt_i) = result(pt_i, eta_i);
      result(pt_i, time_i) = -w * cosA - dw_dpt * px - dw_dpt * dpx_dct * ct;
      result(time_i, pt_i) = result(pt_i, time_i);

      result(eta_i, eta_i) = -d2w_d2eta * px * ct - dw_deta * dpx_deta * ct;
      result(eta_i, time_i) = -dw_deta * px - dw_deta * dpx_dct * ct;
      result(time_i, eta_i) = result(eta_i, time_i);
      result(time_i, time_i) = -w * dpx_dct;
      break;
    case kfbase::core::MOMENT_Z:
      result(pt_i, eta_i) = 1;
      result(eta_i, pt_i) = result(pt_i, eta_i);
      break;
    case kfbase::core::MOMENT_E:
      result(pt_i, pt_i) = d2e_d2pt;
      result(pt_i, eta_i) = d2e_dpt_deta;
      result(eta_i, pt_i) = result(pt_i, eta_i);
      result(eta_i, eta_i) = d2e_d2eta;
      break;
  }
  return result;
}

double kfcmd::core::ChargedParticle::calcVertexComponent(
    const Eigen::VectorXd& x, kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- pt
  // 1 --- ctg theta
  // 2 --- phi
  // 3 --- rho
  // 4 --- tz
  long pt_i = getBeginIndex();
  double result = 0;
  double ct = x(_timeParam->getBeginIndex());
  double m = getMass();
  double eta = x(pt_i + 1);
  double pt = x(pt_i);
  double energy = std::sqrt(m * m + (1 + eta * eta) * pt * pt);
  double qBc = getCharge() * getMagneticField() * _c;
  double r = pt / qBc;
  double w = qBc / energy;
  double phi = x(pt_i + 2);
  double alpha = w * ct - phi;
  double a = r - x(pt_i + 3);
  switch (component) {
    case kfbase::core::VERTEX_X:
      result = a * std::sin(phi) + r * std::sin(alpha);
      break;
    case kfbase::core::VERTEX_Y:
      result = -a * std::cos(phi) + r * std::cos(alpha);
      break;
    case kfbase::core::VERTEX_Z:
      result = x(pt_i + 4) + pt * eta * ct / energy;
      break;
  }

  return result;
}

Eigen::VectorXd kfcmd::core::ChargedParticle::calcDVertexComponent(
    const Eigen::VectorXd& x, kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- pt
  // 1 --- ctg theta
  // 2 --- phi
  // 3 --- rho
  // 4 --- tz
  long pt_i = getBeginIndex();
  const long time_i = _timeParam->getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  double ct = x(_timeParam->getBeginIndex());
  double m = getMass();
  double eta = x(pt_i + 1);
  double eta2 = eta * eta;
  double pt = x(pt_i);
  double pt2 = pt * pt;
  double c0 = 1 + eta2;
  double energy = std::sqrt(m * m + c0 * pt2);
  double qBc = getCharge() * getMagneticField() * _c;
  double r = pt / qBc;
  double w = qBc / energy;
  double phi = x(pt_i + 2);
  double alpha = w * ct - phi;
  double a = r - x(pt_i + 3);
  double dw_de = -w / energy;
  double de_dpt = pt * c0 / energy;
  double de_deta = pt2 * eta / energy;
  double dw_dpt = dw_de * de_dpt;
  double dw_deta = dw_de * de_deta;
  double cosA = std::cos(alpha);
  double sinA = std::sin(alpha);
  double cosP = std::cos(phi);
  double sinP = std::sin(phi);
  double dz = pt * eta * ct / energy;
  switch (component) {
    case kfbase::core::VERTEX_X:
      result(pt_i) = (sinP + sinA) / qBc + r * dw_dpt * ct * cosA;
      result(pt_i + 1) = r * dw_deta * ct * cosA;
      result(pt_i + 2) = a * cosP - r * cosA;
      result(pt_i + 3) = -sinP;
      result(time_i) = w * r * cosA;
      break;
    case kfbase::core::VERTEX_Y:
      result(pt_i) = (cosA - cosP) / qBc - r * dw_dpt * ct * sinA;
      result(pt_i + 1) = -r * dw_deta * ct * sinA;
      result(pt_i + 2) = a * sinP + r * sinA;
      result(pt_i + 3) = cosP;
      result(time_i) = -w * r * sinA;
      break;
    case kfbase::core::VERTEX_Z:
      result(pt_i) = eta * ct / energy - dz * de_dpt / energy;
      result(pt_i + 1) = pt * ct / energy - dz * de_deta / energy;
      result(pt_i + 4) = 1;
      result(time_i) = pt * eta / energy;
      break;
  }
  return result;
}

Eigen::MatrixXd kfcmd::core::ChargedParticle::calcD2VertexComponent(
    const Eigen::VectorXd& x, kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- pt
  // 1 --- ctg theta
  // 2 --- phi
  // 3 --- rho
  // 4 --- tz
  long pt_i = getBeginIndex();
  const long eta_i = pt_i + 1;
  const long phi_i = pt_i + 2;
  const long rho_i = pt_i + 3;
  const long time_i = _timeParam->getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  double ct = x(_timeParam->getBeginIndex());
  double ct2 = ct * ct;
  double m = getMass();
  double eta = x(pt_i + 1);
  double eta2 = eta * eta;
  double pt = x(pt_i);
  double pt2 = pt * pt;
  double c0 = 1 + eta2;
  double energy = std::sqrt(m * m + c0 * pt2);
  double qBc = getCharge() * getMagneticField() * _c;
  double r = pt / qBc;
  double w = qBc / energy;
  double phi = x(pt_i + 2);
  double alpha = w * ct - phi;
  double a = r - x(pt_i + 3);
  double dw_de = -w / energy;
  double d2w_d2e = 2. * w / energy / energy;
  double de_dpt = pt * c0 / energy;
  double de_deta = pt2 * eta / energy;
  double d2e_d2pt = c0 / energy - de_dpt * de_dpt / energy;
  double d2e_dpt_deta = 2. * eta * pt / energy - de_dpt * de_deta / energy;
  double d2e_d2eta = pt2 / energy - de_deta * de_deta / energy;
  double dw_dpt = dw_de * de_dpt;
  double dw_deta = dw_de * de_deta;
  double d2w_d2pt = d2w_d2e * de_dpt * de_dpt + dw_de * d2e_d2pt;
  double d2w_dpt_deta = d2w_d2e * de_dpt * de_deta + dw_de * d2e_dpt_deta;
  double d2w_d2eta = d2w_d2e * de_deta * de_deta + dw_de * d2e_d2eta;
  double cosA = std::cos(alpha);
  double sinA = std::sin(alpha);
  double cosP = std::cos(phi);
  double sinP = std::sin(phi);
  double dz = pt * eta * ct / energy;
  switch (component) {
    case kfbase::core::VERTEX_X:
      result(pt_i, pt_i) = 2. * dw_dpt * ct * cosA / qBc + r * d2w_d2pt * ct * cosA -
	r * dw_dpt * dw_dpt * ct2 * sinA;
      result(pt_i, eta_i) = dw_deta * ct * cosA / qBc +
      	r * d2w_dpt_deta * ct * cosA - r * dw_dpt * dw_deta * ct2 * sinA;
      result(eta_i, pt_i) = result(pt_i, eta_i);
      result(pt_i, phi_i) = (cosP - cosA) / qBc + r * dw_dpt * ct * sinA;
      result(phi_i, pt_i) = result(pt_i, phi_i);
      result(pt_i, time_i) = w * cosA / qBc + r * dw_dpt * cosA -
       	w * r * dw_dpt * ct * sinA;
      result(time_i, pt_i) = result(pt_i, time_i);

      result(eta_i, eta_i) = r * d2w_d2eta * ct * cosA - r * dw_deta * dw_deta * ct2 * sinA;
      result(eta_i, phi_i) = r * dw_deta * ct * sinA;
      result(phi_i, eta_i) = result(eta_i, phi_i);
      result(eta_i, time_i) = r * dw_deta * cosA - w * r * dw_deta * ct * sinA;
      result(time_i, eta_i) = result(eta_i, time_i);

      result(phi_i, phi_i) = -a * sinP - r * sinA;
      result(phi_i, rho_i) = -cosP;
      result(rho_i, phi_i) = result(phi_i, rho_i);
      result(phi_i, time_i) = w * r * sinA;
      result(time_i, phi_i) = result(phi_i, time_i);
      result(time_i, time_i) = -w * w * r * sinA;
      break;
    case kfbase::core::VERTEX_Y:
      result(pt_i, pt_i) = -2. * dw_dpt * ct * sinA / qBc - r * d2w_d2pt * ct * sinA -
	r * dw_dpt * dw_dpt * ct2 * cosA;
      result(pt_i, eta_i) = -dw_deta * ct * sinA / qBc - r * d2w_dpt_deta * ct * sinA -
      	r * dw_dpt * dw_deta * ct2 * cosA;
      result(eta_i, pt_i) = result(pt_i, eta_i);
      result(pt_i, phi_i) = (sinP + sinA) / qBc + r * dw_dpt * ct * cosA;
      result(phi_i, pt_i) = result(pt_i, phi_i);
      result(pt_i, time_i) = -w * sinA / qBc - r * dw_dpt * sinA - r * dw_dpt * w * ct * cosA;
      result(time_i, pt_i) = result(pt_i, time_i);

      result(eta_i, eta_i) = -r * d2w_d2eta * ct * sinA -r * dw_deta * dw_deta * ct2 * cosA;
      result(eta_i, phi_i) = r * dw_deta * ct * cosA;
      result(phi_i, eta_i) = result(eta_i, phi_i);
      result(eta_i, time_i) = -r * dw_deta * sinA - w * r * dw_deta * ct * cosA;
      result(time_i, eta_i) = result(eta_i, time_i);

      result(phi_i, phi_i) = a * cosP - r * cosA;
      result(phi_i, rho_i) = -sinP;
      result(rho_i, phi_i) = result(phi_i, rho_i);
      result(phi_i, time_i) = w * r * cosA;
      result(time_i, phi_i) = result(phi_i, time_i);
      result(time_i, time_i) = -w * w * r * cosA;
      break;
    case kfbase::core::VERTEX_Z:
      result(pt_i, pt_i) = -2. * eta * ct * de_dpt / energy / energy +
      	2. * dz * de_dpt * de_dpt / energy / energy - dz * d2e_d2pt / energy;
      result(pt_i, eta_i) = ct / energy - eta * ct * de_deta / energy / energy -
      	pt * ct * de_dpt / energy / energy + 2. * dz * de_dpt * de_deta / energy / energy -
      	dz * d2e_dpt_deta / energy;
      result(eta_i, pt_i) = result(pt_i, eta_i);
      result(pt_i, time_i) = eta / energy - pt * eta * de_dpt / energy / energy;
      result(time_i, pt_i) = result(pt_i, time_i);

      result(eta_i, eta_i) = -2. * pt * ct * de_deta / energy / energy +
       	2. * dz * de_deta * de_deta / energy / energy - dz * d2e_d2eta / energy;
      result(eta_i, time_i) = pt / energy - pt * eta * de_deta / energy / energy;
      result(time_i, eta_i) = result(eta_i, time_i);
      break;
  }
  return result;
}

void kfcmd::core::ChargedParticle::setMagneticField(const std::string& name) {
  const auto it = getConstants()->find(name);
  if (it == getConstants()->end()) {
    // TO DO : exception
  }
  _magnetFieldIterator = it;
}

void kfcmd::core::ChargedParticle::setTimeParameter(const std::string& name) {
  auto it = getCommonParameters()->find(name);
  if (it == getCommonParameters()->end()) {
    // TO DO : exception
  }
  _timeParam = it->second;
}

double kfcmd::core::ChargedParticle::getMagneticField() const {
  return _magnetFieldIterator->second;
}
