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

kfcmd::core::Photon::Photon(const std::string& name) : kfbase::core::VertexParticle(name, 7) {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
  setLowerLimit(0, 1.e-2);
  setUpperLimit(0, 1.1);
  setLowerLimit(1, 10);
  setUpperLimit(1, 100);
  setPeriod(2, 0, 2 * TMath::Pi());
  setLowerLimit(2, -1000 * TMath::Pi());
  setUpperLimit(2, 1000 * TMath::Pi());
  setLowerLimit(3, -50);
  setUpperLimit(3, 50);
  // setPeriod(4, 0, 2 * TMath::Pi()); // !!!
  setLowerLimit(4, 0.); // !!!
  setUpperLimit(4, TMath::Pi()); // !!!
  setPeriod(5, 0, 2 * TMath::Pi()); // !!!
  setLowerLimit(5, -1000 * TMath::Pi()); // !!!
  setUpperLimit(5, 1000 * TMath::Pi()); // !!!
  setLowerLimit(6, 0.);
  setUpperLimit(6, 200.);
}

kfcmd::core::Photon::~Photon() {}

double kfcmd::core::Photon::calcDirection(const Eigen::VectorXd &x,
                                          kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
  const long bi = getBeginIndex();
  const double theta = x(bi + 4);
  const double phi = x(bi + 5);
  double result = 0;
  switch (component) {
  case kfbase::core::VERTEX_X:
    result = std::sin(theta) * std::cos(phi);
    break;
  case kfbase::core::VERTEX_Y:
    result = std::sin(theta) * std::sin(phi);
    break;
  case kfbase::core::VERTEX_Z:
    result = std::cos(theta);
    break;
  }
  return result;
}

Eigen::VectorXd kfcmd::core::Photon::calcDDirection(const Eigen::VectorXd &x,
                                                    kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
  const long bi = getBeginIndex();
  const long thetaInd = bi + 4;
  const long phiInd = bi + 5;
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
  case kfbase::core::VERTEX_X:
    result(thetaInd) = std::cos(x(thetaInd)) * std::cos(x(phiInd));
    result(phiInd) = -std::sin(x(thetaInd)) * std::sin(x(phiInd));
    break;
  case kfbase::core::VERTEX_Y:
    result(thetaInd) = std::cos(x(thetaInd)) * std::sin(x(phiInd));
    result(phiInd) = std::sin(x(thetaInd)) * std::cos(x(phiInd));
    break;
  case kfbase::core::VERTEX_Z:
    result(thetaInd) = -std::sin(x(thetaInd));
    break;
  }
  return result;
}

Eigen::MatrixXd kfcmd::core::Photon::calcD2Direction(const Eigen::VectorXd& x,
                                                     kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
  const long bi = getBeginIndex();
  const long thetaInd = bi + 4;
  const long phiInd = bi + 5;
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
  const double sinTheta = std::sin(x(thetaInd));
  const double cosTheta = std::cos(x(thetaInd));
  const double sinPhi = std::sin(x(phiInd));
  const double cosPhi = std::cos(x(phiInd));
  switch (component) {
  case kfbase::core::VERTEX_X:
    result(thetaInd, thetaInd) = -sinTheta * cosPhi;
    result(phiInd, phiInd) = result(thetaInd, thetaInd);
    result(thetaInd, phiInd) = -cosTheta * sinPhi;
    result(phiInd, thetaInd) = result(thetaInd, phiInd);
    break;
  case kfbase::core::VERTEX_Y:
    result(thetaInd, thetaInd) = -sinTheta * sinPhi;
    result(phiInd, phiInd) = result(thetaInd, thetaInd);
    result(thetaInd, phiInd) = cosTheta * cosPhi;
    result(phiInd, thetaInd) = result(thetaInd, phiInd);
    break;
  case kfbase::core::VERTEX_Z:
    result(thetaInd, thetaInd) = -cosTheta;
    break;
  }
  return result;
}

double kfcmd::core::Photon::calcConversionPoint(const Eigen::VectorXd &x,
                                                kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
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
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
  const long bi = getBeginIndex();
  Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
  switch (component) {
  case kfbase::core::VERTEX_X:
    result(bi + 1) = std::cos(x(bi + 2));
    result(bi + 2) = -x(bi + 1) * std::sin(x(bi + 2));
    break;
  case kfbase::core::VERTEX_Y:
    result(bi + 1) = std::sin(x(bi + 1));
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
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
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

double kfcmd::core::Photon::calcOutputMomentumComponent(const Eigen::VectorXd& x,
                                                        kfbase::core::MOMENT_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
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

double kfcmd::core::Photon::calcInputMomentumComponent(const Eigen::VectorXd& x,
                                                       kfbase::core::MOMENT_COMPONENT component) const {
  return calcOutputMomentumComponent(x, component);
}

Eigen::VectorXd kfcmd::core::Photon::calcOutputDMomentumComponent(const Eigen::VectorXd& x,
                                                                  kfbase::core::MOMENT_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
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
                                                                 kfbase::core::MOMENT_COMPONENT component) const {
  return calcOutputDMomentumComponent(x, component);
}

Eigen::MatrixXd kfcmd::core::Photon::calcOutputD2MomentumComponent(const Eigen::VectorXd& x,
                                                                   kfbase::core::MOMENT_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
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
                                                                  kfbase::core::MOMENT_COMPONENT component) const {
  return calcOutputD2MomentumComponent(x, component);
}

double kfcmd::core::Photon::calcOutputVertexComponent(const Eigen::VectorXd &x,
                                                      kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
  const long bi = getBeginIndex();
  return calcConversionPoint(x, component) - x(bi + 6) * calcDirection(x, component);
}

double kfcmd::core::Photon::calcInputVertexComponent(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const {
  return 0.;
}

Eigen::VectorXd kfcmd::core::Photon::calcOutputDVertexComponent(const Eigen::VectorXd& x,
                                                                kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
  const long bi = getBeginIndex();
  Eigen::VectorXd result = calcDConversionPoint(x, component) - x(bi + 6) * calcDDirection(x, component);
  result(bi + 6) -= calcDirection(x, component);
  return result;
}

Eigen::VectorXd kfcmd::core::Photon::calcInputDVertexComponent(const Eigen::VectorXd &x,
                                                               kfbase::core::VERTEX_COMPONENT) const {
  return Eigen::VectorXd::Zero(x.size());
}

Eigen::MatrixXd kfcmd::core::Photon::calcOutputD2VertexComponent(const Eigen::VectorXd &x,
                                                                 kfbase::core::VERTEX_COMPONENT component) const {
  // 0 --- energy
  // 1 --- R_c (conversion point coordinate)
  // 2 --- phi_c (conversion point coordinate)
  // 3 --- z0_c (conversion point coordinate)
  // 4 --- theta (momentum direction coordinate)
  // 5 --- phi (momentum direction coordinate)
  // 6 --- ct
  const long bi = getBeginIndex();
  const long timeInd = bi + 6;
  Eigen::MatrixXd result = calcD2ConversionPoint(x, component) - x(timeInd) * calcD2Direction(x, component);
  result.col(timeInd) = -calcDDirection(x, component);
  result.row(timeInd) = result.col(timeInd).transpose();
  return result;
}

Eigen::MatrixXd kfcmd::core::Photon::calcInputD2VertexComponent(const Eigen::VectorXd& x,
                                                                kfbase::core::VERTEX_COMPONENT component) const {
  return Eigen::MatrixXd::Zero(x.size(), x.size());
}

void kfcmd::core::Photon::onFitBegin(const Eigen::VectorXd& x) {
  kfbase::core::VertexParticle::onFitBegin(x);
  initialDirection_ = TVector3(calcDirection(x, kfbase::core::VERTEX_X),
                               calcDirection(x, kfbase::core::VERTEX_Y),
                               calcDirection(x, kfbase::core::VERTEX_Z));
  initialConvPoint_ = TVector3(calcConversionPoint(x, kfbase::core::VERTEX_X),
                               calcConversionPoint(x, kfbase::core::VERTEX_Y),
                               calcConversionPoint(x, kfbase::core::VERTEX_Z));
}

void kfcmd::core::Photon::onFitEnd(const Eigen::VectorXd& x) {
  kfbase::core::VertexParticle::onFitEnd(x);
  finalDirection_ = TVector3(calcDirection(x, kfbase::core::VERTEX_X),
                               calcDirection(x, kfbase::core::VERTEX_Y),
                               calcDirection(x, kfbase::core::VERTEX_Z));
  finalConvPoint_ = TVector3(calcConversionPoint(x, kfbase::core::VERTEX_X),
                               calcConversionPoint(x, kfbase::core::VERTEX_Y),
                               calcConversionPoint(x, kfbase::core::VERTEX_Z));
}

const TVector3& kfcmd::core::Photon::getInitialDirection() const {
  return initialDirection_;
}

const TVector3& kfcmd::core::Photon::getFinalDirection() const {
  return finalDirection_;
}

const TVector3& kfcmd::core::Photon::getInitialConvPoint() const {
  return initialConvPoint_;
}

const TVector3& kfcmd::core::Photon::getFinalConvPoint() const {
  return finalConvPoint_;
}
