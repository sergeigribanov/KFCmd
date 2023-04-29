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
 * @file ParticleEThetaPhi.cpp
 *
 * @brief Implementation of ParticleEThetaPhi methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/core/HardPi0.hpp"

#include <TDatabasePDG.h>
#include <cmath>
#include <iostream>

const double kfcmd::core::HardPi0::pi0Mass_ =
    TDatabasePDG::Instance()->GetParticle(111)->Mass();

kfcmd::core::HardPi0::HardPi0(const std::string &name)
    : kfbase::core::Particle(name, 3) {
  // 0 --- energy
  // 1 --- theta
  // 2 --- phi
  setLowerLimit(0, pi0Mass_);
  setUpperLimit(0, 1.1);
  setLowerLimit(1, 0.);          // !!!
  setUpperLimit(1, TMath::Pi()); // !!!
  setPeriod(2, 0, 2 * TMath::Pi());
  setLowerLimit(2, -1000 * TMath::Pi());
  setUpperLimit(2, 1000 * TMath::Pi());
}

double kfcmd::core::HardPi0::calcOutputMomentumComponent(
    const Eigen::VectorXd &x, kfbase::core::MOMENT_COMPONENT component) const {
  const long bi = getBeginIndex();
  double result = 0;
  // E --- 0
  // theta --- 1
  // phi --- 2
  switch (component) {
    case kfbase::core::MOMENT_X:
    result = std::sqrt(x(bi) * x(bi) - pi0Mass_ * pi0Mass_) *
             std::sin(x(bi + 1)) * std::cos(x(bi + 2));
    break;
    case kfbase::core::MOMENT_Y:
    result = std::sqrt(x(bi) * x(bi) - pi0Mass_ * pi0Mass_) *
             std::sin(x(bi + 1)) * std::sin(x(bi + 2));
    break;
    case kfbase::core::MOMENT_Z:
    result = std::sqrt(x(bi) * x(bi) - pi0Mass_ * pi0Mass_) *
             std::cos(x(bi + 1));
    break;
    case kfbase::core::MOMENT_E:
    result = x(bi);
    break;
  }
  return result;
}

double kfcmd::core::HardPi0::calcInputMomentumComponent(
    const Eigen::VectorXd &x, kfbase::core::MOMENT_COMPONENT component) const {
  return calcOutputMomentumComponent(x, component);
}

Eigen::VectorXd kfcmd::core::HardPi0::calcOutputDMomentumComponent(
    const Eigen::VectorXd &x, kfbase::core::MOMENT_COMPONENT component) const {
  // E --- 0
  // theta --- 1
  // phi --- 2
  const long bi = getBeginIndex();
  const double sinT = std::sin(x(bi + 1));
  const double cosT = std::cos(x(bi + 1));
  const double sinP = std::sin(x(bi + 2));
  const double cosP = std::cos(x(bi + 2));
  const double p = std::sqrt(x(bi) * x(bi) - pi0Mass_ * pi0Mass_);
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
    case kfbase::core::MOMENT_X:
      result(bi) = x(bi) * sinT * cosP / p;
      result(bi + 1) = p * cosT * cosP;
      result(bi + 2) = -p * sinT * sinP;
      break;
    case kfbase::core::MOMENT_Y:
      result(bi) = x(bi) * sinT * sinP / p;
      result(bi + 1) = p * cosT * sinP;
      result(bi + 2) = p * sinT * cosP;
      break;
    case kfbase::core::MOMENT_Z:
      result(bi) = x(bi) * cosT / p;
      result(bi + 1) = -p * sinT;
      break;
    case kfbase::core::MOMENT_E:
      result(bi) = 1.;
      break;
  }
  return result;
}

Eigen::VectorXd kfcmd::core::HardPi0::calcInputDMomentumComponent(
    const Eigen::VectorXd &x, kfbase::core::MOMENT_COMPONENT component) const {
  return calcOutputDMomentumComponent(x, component);
}

Eigen::MatrixXd kfcmd::core::HardPi0::calcOutputD2MomentumComponent(const Eigen::VectorXd &x,
                                                                    kfbase::core::MOMENT_COMPONENT component) const {
  // E --- 0
  // theta --- 1
  // phi --- 2
  const long bi = getBeginIndex();
  const double sinT = std::sin(x(bi + 1));
  const double cosT = std::cos(x(bi + 1));
  const double sinP = std::sin(x(bi + 2));
  const double cosP = std::cos(x(bi + 2));
  const double p = std::sqrt(x(bi) * x(bi) - pi0Mass_ * pi0Mass_);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  switch (component) {
    case kfbase::core::MOMENT_X:
      // x(bi) * sinT * cosP
      result(bi, bi) = (1. / p - x(bi) * x(bi) / std::pow(p, 3)) * sinT * cosP;
      result(bi, bi + 1) = x(bi) * cosT * cosP / p;
      result(bi + 1, bi) = result(bi, bi + 1);
      result(bi, bi + 2) = -x(bi) * sinT * sinP / p;
      result(bi + 2, bi) = result(bi, bi + 2);
      result(bi + 1, bi + 1) = -x(bi) * sinT * cosP;
      result(bi + 1, bi + 2) = -x(bi) * cosT * sinP;
      result(bi + 2, bi + 1) = result(bi + 1, bi + 2);
      result(bi + 2, bi + 2) = -x(bi) * sinT * cosP;
      break;
    case kfbase::core::MOMENT_Y:
      // x(bi) * sinT * sinP
      result(bi, bi) = (1. / p - x(bi) * x(bi) / std::pow(p, 3)) * sinT * sinP;
      result(bi, bi + 1) = x(bi) * cosT * sinP / p;
      result(bi + 1, bi) = result(bi, bi + 1);
      result(bi, bi + 2) = x(bi) * sinT * cosP / p;
      result(bi + 2, bi) = result(bi, bi + 2);
      result(bi + 1, bi + 1) = -x(bi) * sinT * sinP;
      result(bi + 1, bi + 2) = x(bi) * cosT * cosP;
      result(bi + 2, bi + 1) = result(bi + 1, bi + 2);
      result(bi + 2, bi + 2) = -x(bi) * sinT * sinP;
      break;
    case kfbase::core::MOMENT_Z:
      // x(bi) * cosT
      result(bi, bi) = (1. / p - x(bi) * x(bi) / std::pow(p, 3)) * cosT;
      result(bi, bi + 1) = -x(bi) * sinT / p;
      result(bi + 1, bi) = result(bi, bi + 1);
      result(bi + 1, bi + 1) = -x(bi) * cosT;
      break;
    case kfbase::core::MOMENT_E:
      break;
  }
  return result;
}

Eigen::MatrixXd kfcmd::core::HardPi0::calcInputD2MomentumComponent(const Eigen::VectorXd &x,
                                                                   kfbase::core::MOMENT_COMPONENT component) const {
  return calcOutputD2MomentumComponent(x, component);
}
