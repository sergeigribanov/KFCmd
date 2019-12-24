/*
 * KFCmd library
 * See COPYRIGHT file at the top of the source tree.
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
 * @file Photon.cpp
 *
 * @brief Implementation of Photon methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "Photon.hpp"

#include <cmath>

KFCmd::Photon::Photon(const std::string& name) : KFBase::Particle(name, 5) {
  setPeriod(2, 0, 2 * TMath::Pi());
}

KFCmd::Photon::~Photon() {}

double KFCmd::Photon::calcMomentumComponent(
    const Eigen::VectorXd& x, KFBase::MOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  double result = 0;
  // 0 --- energy
  // 1 --- R
  // 2 --- phi
  // 3 --- z0
  // 4 --- theta
  double q = std::sqrt(
      std::pow(x(bi + 3) - x(_vertexZ->getBeginIndex()), 2) +
      std::pow(x(bi + 1) * std::cos(x(bi + 2)) - x(_vertexX->getBeginIndex()),
               2) +
      std::pow(x(bi + 1) * std::sin(x(bi + 2)) - x(_vertexY->getBeginIndex()),
               2));
  switch (component) {
    case KFBase::MOMENT_X:
      // result = x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
      result =
          x(bi) *
          (x(bi + 1) * std::cos(x(bi + 2)) - x(_vertexX->getBeginIndex())) / q;
      break;
    case KFBase::MOMENT_Y:
      // result = x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
      result =
          x(bi) *
          (x(bi + 1) * std::sin(x(bi + 2)) - x(_vertexY->getBeginIndex())) / q;
      break;
    case KFBase::MOMENT_Z:
      // result = x(bi) * std::cos(x(bi + 4));
      result = x(bi) * (x(bi + 3) - x(_vertexZ->getBeginIndex())) / q;
      break;
    case KFBase::MOMENT_E:
      result = x(bi);
      break;
  }
  return result;
}

Eigen::VectorXd KFCmd::Photon::calcDMomentumComponent(
    const Eigen::VectorXd& x, KFBase::MOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  double q = std::sqrt(
      std::pow(x(bi + 3) - x(_vertexZ->getBeginIndex()), 2) +
      std::pow(x(bi + 1) * std::cos(x(bi + 2)) - x(_vertexX->getBeginIndex()),
               2) +
      std::pow(x(bi + 1) * std::sin(x(bi + 2)) - x(_vertexY->getBeginIndex()),
               2));
  double q3 = std::pow(q, 3);
  double sinP = std::sin(x(bi + 2));
  double cosP = std::cos(x(bi + 2));
  long biX = _vertexX->getBeginIndex();
  long biY = _vertexY->getBeginIndex();
  long biZ = _vertexZ->getBeginIndex();
  double vX = x(biX);
  double vY = x(biY);
  double vZ = x(biZ);
  double dx = x(bi + 1) * cosP - vX;
  double dy = x(bi + 1) * sinP - vY;
  double dz = x(bi + 3) - vZ;
  double dqR = x(bi + 1) - vX * cosP - vY * sinP;
  double dqP = x(bi + 1) * (vX * sinP - vY * cosP);
  switch (component) {
    case KFBase::MOMENT_X:
      // // x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
      // result(bi) = std::sin(x(bi + 4)) * std::cos(x(bi + 2));
      // result(bi + 2) = -x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
      // result(bi + 4) = x(bi) * std::cos(x(bi + 4)) * std::cos(x(bi + 2));

      result(bi) = dx / q;
      result(bi + 1) = x(bi) * cosP / q - x(bi) * dx / q3 * dqR;
      result(bi + 2) = -x(bi) * x(bi + 1) * sinP / q - x(bi) * dx * dqP / q3;
      result(bi + 3) = -x(bi) * dx * dz / q3;
      result(biX) = -x(bi) / q + x(bi) * dx * dx / q3;
      result(biY) = x(bi) * dx * dy / q3;
      result(biZ) = -result(bi + 3);
      break;
    case KFBase::MOMENT_Y:
      // //  x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
      // result(bi) = std::sin(x(bi + 4)) * std::sin(x(bi + 2));
      // result(bi + 2) = x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
      // result(bi + 4) = x(bi) * std::cos(x(bi + 4)) * std::sin(x(bi + 2));

      result(bi) = dy / q;
      result(bi + 1) = x(bi) * sinP / q - x(bi) * dy * dqR / q3;
      result(bi + 2) = x(bi) * x(bi + 1) * cosP / q - x(bi) * dy * dqP / q3;
      result(bi + 3) = -x(bi) * dy * dz / q3;
      result(biX) = x(bi) * dx * dy / q3;
      result(biY) = -x(bi) / q + x(bi) * dy * dy / q3;
      result(biZ) = -result(bi + 3);
      break;
    case KFBase::MOMENT_Z:
      // // x(bi) * std::cos(x(bi + 4));
      // result(bi) = std::cos(x(bi + 4));
      // result(bi + 4) = -x(bi) * std::sin(x(bi + 4));

      result(bi) = dz / q;
      result(bi + 1) = -x(bi) * dz * dqR / q3;
      result(bi + 2) = -x(bi) * dz * dqP / q3;
      result(bi + 3) = x(bi) / q - x(bi) * dz * dz / q3;
      result(biX) = x(bi) * dz * dx / q3;
      result(biY) = x(bi) * dz * dy / q3;
      result(biZ) = -result(bi + 3);
      break;
    case KFBase::MOMENT_E:
      result(bi) = 1;
      break;
  }
  return result;
}

Eigen::MatrixXd KFCmd::Photon::calcD2MomentumComponent(
    const Eigen::VectorXd& x, KFBase::MOMENT_COMPONENT component) const {
  long bi = getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  double q = std::sqrt(
      std::pow(x(bi + 3) - x(_vertexZ->getBeginIndex()), 2) +
      std::pow(x(bi + 1) * std::cos(x(bi + 2)) - x(_vertexX->getBeginIndex()),
               2) +
      std::pow(x(bi + 1) * std::sin(x(bi + 2)) - x(_vertexY->getBeginIndex()),
               2));
  double q3 = std::pow(q, 3);
  double q5 = std::pow(q, 5);
  double sinP = std::sin(x(bi + 2));
  double cosP = std::cos(x(bi + 2));
  long biX = _vertexX->getBeginIndex();
  long biY = _vertexY->getBeginIndex();
  long biZ = _vertexZ->getBeginIndex();
  double vX = x(biX);
  double vY = x(biY);
  double vZ = x(biZ);
  double dx = x(bi + 1) * cosP - vX;
  double dy = x(bi + 1) * sinP - vY;
  double dz = x(bi + 3) - vZ;
  double dxdy = dx * dy;
  double dxdz = dx * dz;
  double dydz = dy * dz;
  double dx2 = dx * dx;
  double dy2 = dy * dy;
  double dz2 = dz * dz;
  double dqR = x(bi + 1) - vX * cosP - vY * sinP;
  double dqP = x(bi + 1) * (vX * sinP - vY * cosP);
  switch (component) {
    case KFBase::MOMENT_X:
      // // x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi + 2));
      // result(bi, bi + 2) = -std::sin(x(bi + 4)) * std::sin(x(bi + 2));
      // result(bi + 2, bi) = result(bi, bi + 2);
      // result(bi, bi + 4) = std::cos(x(bi + 4)) * std::cos(x(bi + 2));
      // result(bi + 4, bi) = result(bi, bi + 4);
      // result(bi + 2, bi + 2) = -x(bi) * std::sin(x(bi + 4)) * std::cos(x(bi +
      // 2)); result(bi + 2, bi + 4) = -x(bi) * std::cos(x(bi + 4)) *
      // std::sin(x(bi + 2)); result(bi + 4, bi + 2) = result(bi + 2, bi + 4);
      // result(bi + 4, bi + 4) = result(bi + 2, bi + 2);

      result(bi, bi + 1) = cosP / q - dx * dqR / q3;
      result(bi + 1, bi) = result(bi, bi + 1);
      result(bi, bi + 2) = -x(bi + 1) * sinP / q - dx * dqP / q3;
      result(bi + 2, bi) = result(bi, bi + 2);
      result(bi, bi + 3) = -dxdz / q3;
      result(bi + 3, bi) = result(bi, bi + 3);
      result(bi, biX) = -1. / q + dx2 / q3;
      result(biX, bi) = result(bi, biX);
      result(bi, biY) = dxdy / q3;
      result(biY, bi) = result(bi, biY);
      result(bi, biZ) = -result(bi, bi + 3);
      result(biZ, bi) = result(bi, biZ);
      result(bi + 1, bi + 1) = -2. * x(bi) * cosP * dqR / q3 - x(bi) * dx / q3 +
                               3. * x(bi) * dx * dqR * dqR / q5;
      result(bi + 1, bi + 2) = -x(bi) * sinP / q - x(bi) * cosP * dqP / q3 +
                               x(bi) * x(bi + 1) * sinP * dqR / q3 -
                               x(bi) * dx * (vX * sinP - vY * cosP) / q3 +
                               3. * x(bi) * dx * dqR * dqP / q5;
      result(bi + 2, bi + 1) = result(bi + 1, bi + 2);
      result(bi + 1, bi + 3) =
          -x(bi) * dz * cosP / q3 + 3. * x(bi) * dxdz * dqR / q5;
      result(bi + 3, bi + 1) = result(bi + 1, bi + 3);
      result(bi + 1, biX) = 2. * x(bi) * dx * cosP / q3 + x(bi) * dqR / q3 -
                            3. * x(bi) * dx2 * dqR / q5;
      result(biX, bi + 1) = result(bi + 1, biX);
      result(bi + 1, biY) = x(bi) * dy * cosP / q3 + x(bi) * dx * sinP / q3 -
                            3. * x(bi) * dxdy * dqR / q5;
      result(biY, bi + 1) = result(bi + 1, biY);
      result(bi + 1, biZ) = -result(bi + 1, bi + 3);
      result(biZ, bi + 1) = result(bi + 1, biZ);
      result(bi + 2, bi + 2) =
          -x(bi) * x(bi + 1) * cosP / q +
          2. * x(bi) * x(bi + 1) * dqP * sinP / q3 -
          x(bi) * x(bi + 1) * dx * (vX * cosP + vY * sinP) / q3 +
          3. * x(bi) * dx * dqP * dqP / q5;
      result(bi + 2, bi + 3) =
          x(bi) * x(bi + 1) * dz * sinP / q3 + 3. * x(bi) * dxdz * dqP / q5;
      result(bi + 3, bi + 2) = result(bi + 2, bi + 3);
      result(bi + 2, biX) = -2. * x(bi) * x(bi + 1) * dx * sinP / q3 +
                            x(bi) * dqP / q3 - 3. * x(bi) * dx2 * dqP / q5;
      result(biX, bi + 2) = result(bi + 2, biX);
      result(bi + 2, biY) = -x(bi) * x(bi + 1) * dy * sinP / q3 +
                            x(bi) * x(bi + 1) * dx * cosP / q3 -
                            3. * x(bi) * dxdy * dqP / q5;
      result(biY, bi + 2) = result(bi + 2, biY);
      result(bi + 2, biZ) = -result(bi + 2, bi + 3);
      result(biZ, bi + 2) = result(bi + 2, biZ);
      result(bi + 3, bi + 3) = -x(bi) * dx / q3 + 3. * x(bi) * dx * dz2 / q5;
      result(bi + 3, biX) = x(bi) * dz / q3 - 3. * x(bi) * dx2 * dz / q5;
      result(biX, bi + 3) = result(bi + 3, biX);
      result(bi + 3, biY) = -3. * x(bi) * dxdy * dz / q5;
      result(biY, bi + 3) = result(bi + 3, biY);
      result(bi + 3, biZ) = -result(bi + 3, bi + 3);
      result(biZ, bi + 3) = result(bi + 3, biZ);
      result(biX, biX) = -3. * x(bi) * dx / q3 + 3. * x(bi) * dx2 * dx / q5;
      result(biX, biY) = -x(bi) * dy / q3 + 3. * x(bi) * dx2 * dy / q5;
      result(biY, biX) = result(biX, biY);
      result(biX, biZ) = -result(bi + 3, biX);
      result(biZ, biX) = result(biX, biZ);
      result(biY, biY) = -x(bi) * dx / q3 + 3. * x(bi) * dx * dy2 / q5;
      result(biY, biZ) = -result(bi + 3, biY);
      result(biZ, biY) = result(biY, biZ);
      result(biZ, biZ) = result(bi + 3, bi + 3);
      break;
    case KFBase::MOMENT_Y:
      // //  x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi + 2));
      // result(bi, bi + 2) = std::sin(x(bi + 4)) * std::cos(x(bi + 2));
      // result(bi + 2, bi) = result(bi, bi + 2);
      // result(bi, bi + 4) = std::cos(x(bi + 4)) * std::sin(x(bi + 2));
      // result(bi + 4, bi) = result(bi, bi + 4);
      // result(bi + 2, bi + 2) = -x(bi) * std::sin(x(bi + 4)) * std::sin(x(bi +
      // 2)); result(bi + 2, bi + 4) = x(bi) * std::cos(x(bi + 4)) *
      // std::cos(x(bi + 2)); result(bi + 4, bi + 2) = result(bi + 2, bi + 4);
      // result(bi + 4, bi + 4) = result(bi + 2, bi + 2);

      result(bi, bi + 1) = sinP / q - dy * dqR / q3;
      result(bi + 1, bi) = result(bi, bi + 1);
      result(bi, bi + 2) = x(bi + 1) * cosP / q - dy * dqP / q3;
      result(bi + 2, bi) = result(bi, bi + 2);
      result(bi, bi + 3) = -dydz / q3;
      result(bi + 3, bi) = result(bi, bi + 3);
      result(bi, biX) = dxdy / q3;
      result(biX, bi) = result(bi, biX);
      result(bi, biY) = -1. / q + dy2 / q3;
      result(biY, bi) = result(bi, biY);
      result(bi, biZ) = -result(bi, bi + 3);
      result(biZ, bi) = result(bi, biZ);
      result(bi + 1, bi + 1) = -2. * x(bi) * sinP * dqR / q3 - x(bi) * dy / q3 +
                               3. * x(bi) * dy * dqR * dqR / q5;
      result(bi + 1, bi + 2) = x(bi) * cosP / q - x(bi) * sinP * dqP / q3 -
                               x(bi) * x(bi + 1) * cosP * dqR / q3 -
                               x(bi) * dy * (vX * sinP - vY * cosP) / q3 +
                               3. * x(bi) * dy * dqR * dqP / q5;
      result(bi + 2, bi + 1) = result(bi + 1, bi + 2);
      result(bi + 1, bi + 3) =
          -x(bi) * dz * sinP / q3 + 3. * x(bi) * dydz * dqR / q5;
      result(bi + 3, bi + 1) = result(bi + 1, bi + 3);
      result(bi + 1, biX) = x(bi) * dx * sinP / q3 + x(bi) * dy * cosP / q3 -
                            3. * x(bi) * dxdy * dqR / q5;
      result(biX, bi + 1) = result(bi + 1, biX);
      result(bi + 1, biY) = x(bi) * dy * sinP / q3 + x(bi) * dqR / q3 +
                            x(bi) * dy * sinP / q3 -
                            3. * x(bi) * dy2 * dqR / q5;
      result(biY, bi + 1) = result(bi + 1, biY);
      result(bi + 1, biZ) = -result(bi + 1, bi + 3);
      result(biZ, bi + 1) = result(bi + 1, biZ);
      result(bi + 2, bi + 2) =
          -x(bi) * x(bi + 1) * sinP / q -
          2. * x(bi) * x(bi + 1) * cosP * dqP / q3 -
          x(bi) * dy * x(bi + 1) * (vX * cosP + vY * sinP) / q3 +
          3. * x(bi) * dy * dqP * dqP / q5;
      result(bi + 2, bi + 3) =
          -x(bi) * x(bi + 1) * dz * cosP / q3 + 3. * x(bi) * dydz * dqP / q5;
      result(bi + 3, bi + 2) = result(bi + 2, bi + 3);
      result(bi + 2, biX) = x(bi) * x(bi + 1) * dx * cosP / q3 -
                            x(bi) * x(bi + 1) * dy * sinP / q3 -
                            3. * x(bi) * dxdy * dqP / q5;
      result(biX, bi + 2) = result(bi + 2, biX);
      result(bi + 2, biY) =
          x(bi) * x(bi + 1) * dy * cosP / q3 + x(bi) * dqP / q3 +
          x(bi) * x(bi + 1) * dy * cosP / q3 - 3. * x(bi) * dy2 * dqP / q5;
      result(biY, bi + 2) = result(bi + 2, biY);
      result(bi + 2, biZ) = -result(bi + 2, bi + 3);
      result(biZ, bi + 2) = result(bi + 2, biZ);
      result(bi + 3, bi + 3) = -x(bi) * dy / q3 + 3. * x(bi) * dy * dz2 / q5;
      result(bi + 3, biX) = -3. * x(bi) * dxdy * dz / q5;
      result(biX, bi + 3) = result(bi + 3, biX);
      result(bi + 3, biY) = x(bi) * dz / q3 - 3. * x(bi) * dy2 * dz / q5;
      result(biY, bi + 3) = result(bi + 3, biY);
      result(bi + 3, biZ) = -result(bi + 3, bi + 3);
      result(biZ, bi + 3) = result(bi + 3, biZ);
      result(biX, biX) = -x(bi) * dy / q3 + 3. * x(bi) * dx2 * dy / q5;
      result(biX, biY) = -x(bi) * dx / q3 + 3. * x(bi) * dx * dy2 / q5;
      result(biY, biX) = result(biX, biY);
      result(biX, biZ) = -result(bi + 3, biX);
      result(biZ, biX) = result(biX, biZ);
      result(biY, biY) = -3. * x(bi) * dy / q3 + 3. * x(bi) * dy2 * dy / q5;
      result(biY, biZ) = -result(bi + 3, biY);
      result(biZ, biY) = result(biY, biZ);
      result(biZ, biZ) = result(bi + 3, bi + 3);
      break;
    case KFBase::MOMENT_Z:
      // // x(bi) * std::cos(x(bi + 4));
      // result(bi, bi + 4) = -std::sin(x(bi + 4));
      // result(bi + 4, bi) = result(bi, bi + 4);
      // result(bi + 4, bi + 4) = -x(bi) * std::cos(x(bi + 4));

      result(bi, bi + 1) = -dz * dqR / q3;
      result(bi + 1, bi) = result(bi, bi + 1);
      result(bi, bi + 2) = -dz * dqP / q3;
      result(bi + 2, bi) = result(bi, bi + 2);
      result(bi, bi + 3) = 1. / q - dz2 / q3;
      result(bi + 3, bi) = result(bi, bi + 3);
      result(bi, biX) = dxdz / q3;
      result(biX, bi) = result(bi, biX);
      result(bi, biY) = dydz / q3;
      result(biY, bi) = result(bi, biY);
      result(bi, biZ) = -result(bi, bi + 3);
      result(biZ, bi) = result(bi, biZ);
      result(bi + 1, bi + 1) =
          -x(bi) * dz / q3 + 3. * x(bi) * dz * dqR * dqR / q5;
      result(bi + 1, bi + 2) = -x(bi) * dz * (vX * sinP - vY * cosP) / q3 +
                               3. * x(bi) * dz * dqR * dqP / q5;
      result(bi + 2, bi + 1) = result(bi + 1, bi + 2);
      result(bi + 1, bi + 3) = -x(bi) * dqR / q3 + 3. * x(bi) * dz2 * dqR / q5;
      result(bi + 3, bi + 1) = result(bi + 1, bi + 3);
      result(bi + 1, biX) =
          x(bi) * dz * cosP / q3 - 3. * x(bi) * dxdz * dqR / q5;
      result(biX, bi + 1) = result(bi + 1, biX);
      result(bi + 1, biY) =
          x(bi) * dz * sinP / q3 - 3. * x(bi) * dydz * dqR / q5;
      result(biY, bi + 1) = result(bi + 1, biY);
      result(bi + 1, biZ) = -result(bi + 3, bi + 1);
      result(biZ, bi + 1) = result(bi + 1, biZ);
      result(bi + 2, bi + 2) =
          -x(bi) * x(bi + 1) * dz * (vX * cosP + vY * sinP) / q3 +
          3. * x(bi) * dz * dqP * dqP / q5;
      result(bi + 2, bi + 3) = -x(bi) * dqP / q3 + 3. * x(bi) * dz2 * dqP / q5;
      result(bi + 3, bi + 2) = result(bi + 2, bi + 3);
      result(bi + 2, biX) =
          -x(bi) * x(bi + 1) * dz * sinP / q3 - 3. * x(bi) * dxdz * dqP / q5;
      result(biX, bi + 2) = result(bi + 2, biX);
      result(bi + 2, biY) =
          x(bi) * x(bi + 1) * dz * cosP / q3 - 3. * x(bi) * dydz * dqP / q5;
      result(biY, bi + 2) = result(bi + 2, biY);
      result(bi + 2, biZ) = -result(bi + 2, bi + 3);
      result(biZ, bi + 2) = result(bi + 2, biZ);
      result(bi + 3, bi + 3) =
          -3. * x(bi) * dz / q3 + 3. * x(bi) * dz2 * dz / q5;
      result(bi + 3, biX) = x(bi) * dx / q3 - 3. * x(bi) * dx * dz2 / q5;
      result(biX, bi + 3) = result(bi + 3, biX);
      result(bi + 3, biY) = x(bi) * dy / q3 - 3. * x(bi) * dy * dz2 / q5;
      result(biY, bi + 3) = result(bi + 3, biY);
      result(bi + 3, biZ) = -result(bi + 3, bi + 3);
      result(biZ, bi + 3) = result(bi + 3, biZ);
      result(biX, biX) = -x(bi) * dz / q3 + 3. * x(bi) * dx2 * dz / q5;
      result(biX, biY) = 3. * x(bi) * dxdy * dz / q5;
      result(biY, biX) = result(biX, biY);
      result(biX, biZ) = -result(bi + 3, biX);
      result(biZ, biX) = result(biX, biZ);
      result(biY, biY) = -x(bi) * dz / q3 + 3. * x(bi) * dy2 * dz / q5;
      result(biY, biZ) = -result(bi + 3, biY);
      result(biZ, biY) = result(biY, biZ);
      result(biZ, biZ) = result(bi + 3, bi + 3);
      break;
    case KFBase::MOMENT_E:
      break;
  }
  return result;
}

void KFCmd::Photon::setVertexX(const std::string& name) {
  auto it = getCommonParameters()->find(name);
  if (it == getCommonParameters()->end()) {
    // TO DO : exception
  }
  _vertexX = it->second;
}

void KFCmd::Photon::setVertexY(const std::string& name) {
  auto it = getCommonParameters()->find(name);
  if (it == getCommonParameters()->end()) {
    // TO DO : exception
  }
  _vertexY = it->second;
}

void KFCmd::Photon::setVertexZ(const std::string& name) {
  auto it = getCommonParameters()->find(name);
  if (it == getCommonParameters()->end()) {
    // TO DO : exception
  }
  _vertexZ = it->second;
}
