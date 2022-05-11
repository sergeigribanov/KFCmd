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

#include "kfcmd/core/Photon.hpp"

#include <cmath>
#include <iostream>

kfcmd::core::Photon::Photon(const std::string& name) : kfbase::core::Particle(name, 4) {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  setLowerLimit(0, 1.e-2);
  setUpperLimit(0, 1.1);
  setLowerLimit(1, 10);
  setUpperLimit(1, 100);
  setPeriod(2, 0, 2 * TMath::Pi());
  setLowerLimit(2, -1000 * TMath::Pi());
  setUpperLimit(2, 1000 * TMath::Pi());
  setLowerLimit(3, -50);
  setUpperLimit(3, 50);
}

kfcmd::core::Photon::~Photon() {}

double kfcmd::core::Photon::calcConversionPoint(const Eigen::VectorXd &x,
                                                kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  const long bi = getBeginIndex();
  double result = 0;
  switch (component) {
  case kfbase::core::VERTEX_X:
    result = x(bi + 1) * std::cos(x(bi + 2));
    break;
  case kfbase::core::VERTEX_Y:
    result = x(bi + 1) * std::sin(x(bi + 2));
    break;
  case kfbase::core::VERTEX_Z:
    result = x(bi + 3);
    break;
  }
  return result;
}

Eigen::VectorXd kfcmd::core::Photon::calcDConversionPoint(const Eigen::VectorXd &x,
                                                          kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  const long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
  case kfbase::core::VERTEX_X:
    result(bi + 1) = std::cos(x(bi + 2));
    result(bi + 2) = -x(bi + 1) * std::sin(x(bi + 2));
    break;
  case kfbase::core::VERTEX_Y:
    result(bi + 1) = std::sin(x(bi + 2));
    result(bi + 2) = x(bi + 1) * std::cos(x(bi + 2));
    break;
  case kfbase::core::VERTEX_Z:
    result(bi + 3) = 1.;
    break;
  }
  return result;
}

Eigen::MatrixXd kfcmd::core::Photon::calcD2ConversionPoint(const Eigen::VectorXd& x,
                                                           kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  const long bi = getBeginIndex();
  const long rhoInd = bi + 1;
  const long phiInd = bi + 2;
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  switch (component) {
  case kfbase::core::VERTEX_X:
    result(phiInd, phiInd) = -x(rhoInd) * std::cos(x(phiInd));
    result(rhoInd, phiInd) = -std::sin(x(phiInd));
    result(phiInd, rhoInd) = result(rhoInd, phiInd);
    break;
  case kfbase::core::VERTEX_Y:
    result(phiInd, phiInd) = -x(rhoInd) * std::sin(x(phiInd));
    result(rhoInd, phiInd) = std::cos(x(phiInd));
    result(phiInd, rhoInd) = result(rhoInd, phiInd);
    break;
  case kfbase::core::VERTEX_Z:
    break;
  }
  return result;
}

double kfcmd::core::Photon::calcDirection(const Eigen::VectorXd& x,
                                          kfbase::core::VERTEX_COMPONENT component) const {
  Eigen::Vector3d vi;
  vi <<
    calcConversionPoint(x, kfbase::core::VERTEX_X) -
    vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_X),
    calcConversionPoint(x, kfbase::core::VERTEX_Y) -
    vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Y),
    calcConversionPoint(x, kfbase::core::VERTEX_Z) -
    vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Z);
  vi.normalize();
  double result = 0;
  switch (component) {
  case kfbase::core::VERTEX_X:
    result = vi(0);
    break;
  case kfbase::core::VERTEX_Y:
    result = vi(1);
    break;
  case kfbase::core::VERTEX_Z:
    result = vi(2);
    break;
  }
  return result;
}

Eigen::VectorXd kfcmd::core::Photon::calcDDirection(const Eigen::VectorXd& x, kfbase::core::VERTEX_COMPONENT component) const {
  Eigen::Vector3d vi;
  vi <<
    calcConversionPoint(x, kfbase::core::VERTEX_X) -
    vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_X),
    calcConversionPoint(x, kfbase::core::VERTEX_Y) -
    vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Y),
    calcConversionPoint(x, kfbase::core::VERTEX_Z) -
    vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Z);
  const double vnorm = vi.norm();
  vi /= vnorm;
  Eigen::MatrixXd dvi(x.size(), 3);
  dvi <<
    calcDConversionPoint(x, kfbase::core::VERTEX_X) -
    vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_X),
    calcDConversionPoint(x, kfbase::core::VERTEX_Y) -
    vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_Y),
    calcDConversionPoint(x, kfbase::core::VERTEX_Z) -
    vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_Z);
  dvi /= vnorm;
  const Eigen::VectorXd tv = dvi * vi;
  const int index = int(component);
  return dvi.col(index) - vi(index) * tv;
}

Eigen::MatrixXd kfcmd::core::Photon::calcD2Direction(const Eigen::VectorXd& x,
                                                     kfbase::core::VERTEX_COMPONENT component) const {
  Eigen::Vector3d vi;
  vi <<
    calcConversionPoint(x, kfbase::core::VERTEX_X) -
    vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_X),
    calcConversionPoint(x, kfbase::core::VERTEX_Y) -
    vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Y),
    calcConversionPoint(x, kfbase::core::VERTEX_Z) -
    vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Z);
  const double vnorm = vi.norm();
  vi /= vnorm;
  Eigen::MatrixXd dvi(x.size(), 3);
  dvi <<
    calcDConversionPoint(x, kfbase::core::VERTEX_X) -
    vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_X),
    calcDConversionPoint(x, kfbase::core::VERTEX_Y) -
    vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_Y),
    calcDConversionPoint(x, kfbase::core::VERTEX_Z) -
    vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_Z);
  dvi /= vnorm;
  const Eigen::VectorXd tv = dvi * vi;
  const int index = int(component);
  std::vector<Eigen::MatrixXd> d2vi = {calcD2ConversionPoint(x, kfbase::core::VERTEX_X) -
                                       vertex_->calcD2CartesianCoordinate(x, kfbase::core::VERTEX_X),
                                       calcD2ConversionPoint(x, kfbase::core::VERTEX_Y) -
                                       vertex_->calcD2CartesianCoordinate(x, kfbase::core::VERTEX_Y),
                                       calcD2ConversionPoint(x, kfbase::core::VERTEX_Z) -
                                       vertex_->calcD2CartesianCoordinate(x, kfbase::core::VERTEX_Z)};
  for (int i = 0; i < 3; ++i) {
    d2vi[i] /= vnorm;
  }
  Eigen::MatrixXd result = d2vi[index];
  result -= dvi.col(index) * tv.transpose() + tv * dvi.col(index).transpose();
  result -= vi(index) * (vi(0) * d2vi[0] + vi(1) * d2vi[1] + vi(2) * d2vi[2] + dvi * dvi.transpose());
  result += 3. * vi(index) * tv * tv.transpose();
  return result;
}

double kfcmd::core::Photon::calcOutputMomentumComponent(const Eigen::VectorXd& x,
                                                        kfbase::core::MOMENT_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  const long bi = getBeginIndex();
  double result = 0;
  switch (component) {
  case kfbase::core::MOMENT_X:
    result = x(bi) * calcDirection(x, kfbase::core::VERTEX_X);
    break;
  case kfbase::core::MOMENT_Y:
    result = x(bi) * calcDirection(x, kfbase::core::VERTEX_Y);
    break;
  case kfbase::core::MOMENT_Z:
    result = x(bi) * calcDirection(x, kfbase::core::VERTEX_Z);
    break;
  case kfbase::core::MOMENT_E:
    result = x(bi);
    break;
  }
  return result;
}

double kfcmd::core::Photon::calcInputMomentumComponent(const Eigen::VectorXd&,
                                                       kfbase::core::MOMENT_COMPONENT) const {
  return 0.;
}

Eigen::VectorXd kfcmd::core::Photon::calcOutputDMomentumComponent(const Eigen::VectorXd& x,
                                                                  kfbase::core::MOMENT_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  const long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
  case kfbase::core::MOMENT_X:
    result(bi) = calcDirection(x, kfbase::core::VERTEX_X);
    result += x(bi) * calcDDirection(x, kfbase::core::VERTEX_X);
    break;
  case kfbase::core::MOMENT_Y:
    result(bi) = calcDirection(x, kfbase::core::VERTEX_Y);
    result += x(bi) * calcDDirection(x, kfbase::core::VERTEX_Y);
    break;
  case kfbase::core::MOMENT_Z:
    result(bi) = calcDirection(x, kfbase::core::VERTEX_Z);
    result += x(bi) * calcDDirection(x, kfbase::core::VERTEX_Z);
    break;
  case kfbase::core::MOMENT_E:
    result(bi) = 1.;
    break;
  }
  return result;
}

Eigen::VectorXd kfcmd::core::Photon::calcInputDMomentumComponent(const Eigen::VectorXd& x,
                                                                 kfbase::core::MOMENT_COMPONENT) const {
  return Eigen::VectorXd::Zero(x.size());
}

Eigen::MatrixXd kfcmd::core::Photon::calcOutputD2MomentumComponent(const Eigen::VectorXd& x,
                                                                   kfbase::core::MOMENT_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  const long bi = getBeginIndex();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  switch (component) {
  case kfbase::core::MOMENT_X:
    result.col(bi) = calcDDirection(x, kfbase::core::VERTEX_X);
    result.row(bi) = result.col(bi).transpose();
    result += x(bi) * calcD2Direction(x, kfbase::core::VERTEX_X);
    break;
  case kfbase::core::MOMENT_Y:
    result.col(bi) = calcDDirection(x, kfbase::core::VERTEX_Y);
    result.row(bi) = result.col(bi).transpose();
    result += x(bi) * calcD2Direction(x, kfbase::core::VERTEX_Y);
    break;
  case kfbase::core::MOMENT_Z:
    result.col(bi) = calcDDirection(x, kfbase::core::VERTEX_Z);
    result.row(bi) = result.col(bi).transpose();
    result += x(bi) * calcD2Direction(x, kfbase::core::VERTEX_Z);
    break;
  case kfbase::core::MOMENT_E:
    break;
  }
  return result;
}

Eigen::MatrixXd kfcmd::core::Photon::calcInputD2MomentumComponent(const Eigen::VectorXd& x,
                                                                  kfbase::core::MOMENT_COMPONENT) const {
  return Eigen::MatrixXd::Zero(x.size(), x.size());
}

void kfcmd::core::Photon::setOutputVertex(kfbase::core::Vertex *vertex) {
  vertex_ = vertex;
}
