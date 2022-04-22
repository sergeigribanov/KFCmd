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
 * @file Hypothesis.cpp
 *
 * @brief Implementation of Hypothesis methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include <algorithm>
#include "kfcmd/core/Hypothesis.hpp"
#include "kfcmd/core/ParticlePxPyPzE.hpp"
#include "kfcmd/core/ParticleMassLessThetaPhiE.hpp"

#include <kfbase/core/MassConstraint.hpp>
#include <kfbase/core/MomentumConstraint.hpp>
#include <kfbase/core/DoubleParticleAngularConstraint.hpp>
#include <kfbase/core/ParticleAngularConstraint.hpp>
#include <kfbase/core/IntermediateNeutralParticle.hpp>
#include <kfbase/newtonian_opt/CommonParams.hpp>
#include <cmath>

namespace nopt = kfbase::newtonian_opt;

kfcmd::core::Hypothesis::Hypothesis(double energy, double magneticField, long nIter,
                                    double tolerance)
  : kfbase::core::Hypothesis(nIter, tolerance), _energy(energy) {
  addConstant("#m-field", magneticField);
  addConstant("#beam-x", 0.);
  addConstant("#beam-y", 0.);
}

kfcmd::core::Hypothesis::~Hypothesis() {}

void kfcmd::core::Hypothesis::addEnergyConstraint(const std::string& name,
                                                     const std::set<kfbase::core::Particle*>& inputs,
                                                     const std::set<kfbase::core::Particle*>& outputs) {
  const std::string scpe = "#momentum-constraint-" + name + "-pe";
  auto cpe = new kfbase::core::MomentumConstraint(scpe, kfbase::core::MOMENT_E);
  addConstraint(cpe);
  for (auto el : inputs) {
    cpe->inAdd(el);
  }
  for (auto el : outputs) {
    cpe->outAdd(el);
  }
  enableConstraint(scpe);
}

void kfcmd::core::Hypothesis::addMomentumConstraints(const std::string& name,
                                                     const std::set<kfbase::core::Particle*>& inputs,
                                                     const std::set<kfbase::core::Particle*>& outputs) {
  const std::string scpx = "#momentum-constraint-" + name + "-px";
  const std::string scpy = "#momentum-constraint-" + name + "-py";
  const std::string scpz = "#momentum-constraint-" + name + "-pz";
  auto cpx = new kfbase::core::MomentumConstraint(scpx, kfbase::core::MOMENT_X);
  addConstraint(cpx);
  auto cpy = new kfbase::core::MomentumConstraint(scpy, kfbase::core::MOMENT_Y);
  addConstraint(cpy);
  auto cpz = new kfbase::core::MomentumConstraint(scpz, kfbase::core::MOMENT_Z);
  addConstraint(cpz);
  for (auto el : inputs) {
    cpx->inAdd(el);
    cpy->inAdd(el);
    cpz->inAdd(el);
  }
  for (auto el : outputs) {
    cpx->outAdd(el);
    cpy->outAdd(el);
    cpz->outAdd(el);
  }

  enableConstraint(scpx);
  enableConstraint(scpy);
  enableConstraint(scpz);
}

void kfcmd::core::Hypothesis::addEnergyMomentumConstraints(const std::string& name,
                                                           const std::set<kfbase::core::Particle*>& inputs,
                                                           const std::set<kfbase::core::Particle*>& outputs) {
  const std::string scpx = "#momentum-constraint-" + name + "-px";
  const std::string scpy = "#momentum-constraint-" + name + "-py";
  const std::string scpz = "#momentum-constraint-" + name + "-pz";
  const std::string scpe = "#momentum-constraint-" + name + "-pe";
  auto cpx = new kfbase::core::MomentumConstraint(scpx, kfbase::core::MOMENT_X);
  addConstraint(cpx);
  auto cpy = new kfbase::core::MomentumConstraint(scpy, kfbase::core::MOMENT_Y);
  addConstraint(cpy);
  auto cpz = new kfbase::core::MomentumConstraint(scpz, kfbase::core::MOMENT_Z);
  addConstraint(cpz);
  auto cpe = new kfbase::core::MomentumConstraint(scpe, kfbase::core::MOMENT_E);
  addConstraint(cpe);
  for (auto el : inputs) {
    cpx->inAdd(el);
    cpy->inAdd(el);
    cpz->inAdd(el);
    cpe->inAdd(el);
  }
  for (auto el : outputs) {
    cpx->outAdd(el);
    cpy->outAdd(el);
    cpz->outAdd(el);
    cpe->outAdd(el);
  }


  enableConstraint(scpx);
  enableConstraint(scpy);
  enableConstraint(scpz);
  enableConstraint(scpe);
}

void kfcmd::core::Hypothesis::enableEnergyMomentumConstraints(const std::string& name) {
  const std::string scpx = "#momentum-constraint-" + name + "-px";
  const std::string scpy = "#momentum-constraint-" + name + "-py";
  const std::string scpz = "#momentum-constraint-" + name + "-pz";
  const std::string scpe = "#momentum-constraint-" + name + "-pe";
  enableConstraint(scpx);
  enableConstraint(scpy);
  enableConstraint(scpz);
  enableConstraint(scpe);
}

void kfcmd::core::Hypothesis::disableEnergyMomentumConstraints(const std::string& name) {
  const std::string scpx = "#momentum-constraint-" + name + "-px";
  const std::string scpy = "#momentum-constraint-" + name + "-py";
  const std::string scpz = "#momentum-constraint-" + name + "-pz";
  const std::string scpe = "#momentum-constraint-" + name + "-pe";
  disableConstraint(scpx);
  disableConstraint(scpy);
  disableConstraint(scpz);
  disableConstraint(scpe);
}

void kfcmd::core::Hypothesis::setInitialVertexX(const std::string& vertexName,
                                               double x) {
  const std::string vx = "#" + vertexName + "-x";
  Eigen::VectorXd v(1);
  v(0) = x;
  setInitialCommonParams(vx, v);
}

void kfcmd::core::Hypothesis::setInitialVertexY(const std::string& vertexName,
                                                double y) {
  const std::string vy = "#" + vertexName + "-y";
  Eigen::VectorXd v(1);
  v(0) = y;
  setInitialCommonParams(vy, v);
}

void kfcmd::core::Hypothesis::setInitialVertexZ(const std::string& vertexName,
                                                double z) {
  const std::string vz = "#" + vertexName + "-z";
  Eigen::VectorXd v(1);
  v(0) = z;
  setInitialCommonParams(vz, v);
}

void kfcmd::core::Hypothesis::setInitialVertex(const std::string& vertexName,
                                               const Eigen::Vector3d& vertex) {
  setInitialVertexX(vertexName, vertex(0));
  setInitialVertexY(vertexName, vertex(1));
  setInitialVertexZ(vertexName, vertex(2));
}

void kfcmd::core::Hypothesis::setInitialVertexRandomly(const std::string& vertexName, double distance) {
  Eigen::Vector3d vertex = distance * Eigen::Vector3d::Random();
  setInitialVertex(vertexName, vertex);
}

void kfcmd::core::Hypothesis::addVertex(const std::string& vertexName) {
  if (_vertices.find(vertexName) != _vertices.end()) {
    // TO DO : exception;
    return;
  }
  const std::string vx = "#" + vertexName + "-x";
  const std::string vy = "#" + vertexName + "-y";
  const std::string vz = "#" + vertexName + "-z";
  auto xvertex = new nopt::CommonParams(vx, 1);
  xvertex->setLowerLimit(0, -30);
  xvertex->setUpperLimit(0, 30);
  auto yvertex = new nopt::CommonParams(vy, 1);
  yvertex->setLowerLimit(0, -30);
  yvertex->setUpperLimit(0, 30);
  auto zvertex = new nopt::CommonParams(vz, 1);
  zvertex->setLowerLimit(0, -20);
  zvertex->setUpperLimit(0, 20);
  addCommonParams(xvertex);
  addCommonParams(yvertex);
  addCommonParams(zvertex);
  enableCommonParams(vx);
  enableCommonParams(vy);
  enableCommonParams(vz);
  _vertices.insert(vertexName);
}

void kfcmd::core::Hypothesis::disableVertex(const std::string& vertexName) {
  disableCommonParams("#" + vertexName + "-x");
  disableCommonParams("#" + vertexName + "-y");
  disableCommonParams("#" + vertexName + "-z");
}

void kfcmd::core::Hypothesis::enableVertex(const std::string& vertexName) {
  enableCommonParams("#" + vertexName + "-x");
  enableCommonParams("#" + vertexName + "-y");
  enableCommonParams("#" + vertexName + "-z");
}

void kfcmd::core::Hypothesis::disableVertexComponent(
                                                     const std::string& vertexName, kfbase::core::VERTEX_COMPONENT component) {
  switch (component) {
  case kfbase::core::VERTEX_X:
    disableCommonParams("#" + vertexName + "-x");
    break;
  case kfbase::core::VERTEX_Y:
    disableCommonParams("#" + vertexName + "-y");
    break;
  case kfbase::core::VERTEX_Z:
    disableCommonParams("#" + vertexName + "-z");
    break;
  }
}

void kfcmd::core::Hypothesis::enableVertexComponent(
                                                    const std::string& vertexName, kfbase::core::VERTEX_COMPONENT component) {
  switch (component) {
  case kfbase::core::VERTEX_X:
    enableCommonParams("#" + vertexName + "-x");
    break;
  case kfbase::core::VERTEX_Y:
    enableCommonParams("#" + vertexName + "-y");
    break;
  case kfbase::core::VERTEX_Z:
    enableCommonParams("#" + vertexName + "-z");
    break;
  }
}

TVector3 kfcmd::core::Hypothesis::calcVertexComponent(const std::string& chargedParticleName) {
  const auto& particle = dynamic_cast<kfcmd::core::ChargedParticle*>(_particles.at(chargedParticleName));
  Eigen::VectorXd x(_opt.getN());
  x.segment(particle->getBeginIndex(), particle->getN()) = particle->getBeginParameters();
  TVector3 result;
  result(0) = particle->calcVertexComponent(x, kfbase::core::VERTEX_X);
  result(1) = particle->calcVertexComponent(x, kfbase::core::VERTEX_Y);
  result(2) = particle->calcVertexComponent(x, kfbase::core::VERTEX_Z);
  return result;
}

void kfcmd::core::Hypothesis::fixVertexComponent(const std::string& vertexName,
                                                 double value,
                                                 kfbase::core::VERTEX_COMPONENT component) {
  switch (component) {
  case kfbase::core::VERTEX_X:
    _commonParams.at("#" + vertexName + "-x")->fixParameter(0, value);
    break;
  case kfbase::core::VERTEX_Y:
    _commonParams.at("#" + vertexName + "-y")->fixParameter(0, value);
    break;
  case kfbase::core::VERTEX_Z:
    _commonParams.at("#" + vertexName + "-z")->fixParameter(0, value);
    break;
  }
}

void kfcmd::core::Hypothesis::releaseVertexComponent(
                                                     const std::string& vertexName, kfbase::core::VERTEX_COMPONENT component) {
  switch (component) {
  case kfbase::core::VERTEX_X:
    _commonParams.at("#" + vertexName + "-x")->releaseParameter(0);
    break;
  case kfbase::core::VERTEX_Y:
    _commonParams.at("#" + vertexName + "-y")->releaseParameter(0);
    break;
  case kfbase::core::VERTEX_Z:
    _commonParams.at("#" + vertexName + "-z")->releaseParameter(0);
    break;
  }
}

void kfcmd::core::Hypothesis::addChargedParticle(kfcmd::core::ChargedParticle* particle) {
  addParticle(particle);
  particle->setMagneticField("#m-field");
  particle->setBeamX("#beam-x");
  particle->setBeamY("#beam-y");
}

void kfcmd::core::Hypothesis::addPhoton(kfcmd::core::Photon* photon,
                                        const std::string& vertexName) {
  addParticle(photon);
  const std::string vertexXname = "#" + vertexName + "-x";
  photon->setVertexX(vertexXname);
  const std::string vertexYname = "#" + vertexName + "-y";
  photon->setVertexY(vertexYname);
  const std::string vertexZname = "#" + vertexName + "-z";
  photon->setVertexZ(vertexZname);
}

void kfcmd::core::Hypothesis::addConstantMomentumParticle(const std::string& name,
                                                          double energy,
                                                          const Eigen::Vector3d& p) {
  auto particle = new kfbase::core::ConstantMomentumParticle(name, energy, p);
  addParticle(particle);
}

void kfcmd::core::Hypothesis::addIntermediateNeutralParticle(const std::string& name,
                                                             double mass,
                                                             const std::string& vertexName) {
  auto particle = new kfbase::core::IntermediateNeutralParticle(name, mass);
  addParticle(particle);
  const std::string vertexXname = "#" + vertexName + "-x";
  particle->setVertexX(vertexXname);
  const std::string vertexYname = "#" + vertexName + "-y";
  particle->setVertexY(vertexYname);
  const std::string vertexZname = "#" + vertexName + "-z";
  particle->setVertexZ(vertexZname);
}

void kfcmd::core::Hypothesis::addParticlePxPyPzE(const std::string& name, double mass) {
  auto particle = new kfcmd::core::ParticlePxPyPzE(name, mass);
  addParticle(particle);
}

void kfcmd::core::Hypothesis::addParticleMassLessThetaPhiE(const std::string& name) {
  auto particle = new kfcmd::core::ParticleMassLessThetaPhiE(name);
  addParticle(particle);
}

void kfcmd::core::Hypothesis::addVertexConstraintsXYZ(
                                                      const std::string& chargedParticleName, const std::string& vertexName) {
  auto vtxX = new kfbase::core::VertexConstraint(
                                                 "#" + chargedParticleName + "-constraint-x", kfbase::core::VERTEX_X);
  addConstraint(vtxX);
  const std::string vertexXname = "#" + vertexName + "-x";
  vtxX->setVertexCommonParams(vertexXname);
  auto vtxY = new kfbase::core::VertexConstraint(
                                                 "#" + chargedParticleName + "-constraint-y", kfbase::core::VERTEX_Y);
  addConstraint(vtxY);
  const std::string vertexYname = "#" + vertexName + "-y";
  vtxY->setVertexCommonParams(vertexYname);
  auto vtxZ = new kfbase::core::VertexConstraint(
                                                 "#" + chargedParticleName + "-constraint-z", kfbase::core::VERTEX_Z);
  addConstraint(vtxZ);
  const std::string vertexZname = "#" + vertexName + "-z";
  vtxZ->setVertexCommonParams(vertexZname);
  addParticleToConstraint(chargedParticleName, vtxX->getName());
  addParticleToConstraint(chargedParticleName, vtxY->getName());
  addParticleToConstraint(chargedParticleName, vtxZ->getName());

  _constraints.at(vtxX->getName())->includeUsedCommonParameter(vertexXname);
  _constraints.at(vtxY->getName())->includeUsedCommonParameter(vertexYname);
  _constraints.at(vtxZ->getName())->includeUsedCommonParameter(vertexZname);

  enableConstraint(vtxX->getName());
  enableConstraint(vtxY->getName());
  enableConstraint(vtxZ->getName());
}

void kfcmd::core::Hypothesis::addDoubleParticleAngularConstraint(const std::string& constraintName,
                                                                 const std::string& firstParticle,
                                                                 const std::string& secondParticle,
                                                                 double sigma) {
  auto angC = new kfbase::core::DoubleParticleAngularConstraint(constraintName);
  angC->setLambda(1. / sigma / sigma);
  addConstraint(angC);
  addParticleToConstraint(firstParticle, angC->getName());
  addParticleToConstraint(secondParticle, angC->getName());
  enableConstraint(angC->getName());
}

void kfcmd::core::Hypothesis::addParticleAngularConstraint(const std::string& constraintName,
                                                           const std::string& particleName,
                                                           double sigma) {
  auto angC = new kfbase::core::ParticleAngularConstraint(constraintName);
  angC->setLambda(1. / sigma / sigma);
  addConstraint(angC);
  addParticleToConstraint(particleName, angC->getName());
  enableConstraint(angC->getName());
}

void kfcmd::core::Hypothesis::setParticleAngularConstraintAxis(
                                                               const std::string& constraintName,
                                                               const TVector3& axis) {
  auto cnt = dynamic_cast<kfbase::core::ParticleAngularConstraint*>(_constraints.at(constraintName));
  // TODO: wrong class exception
  cnt->setAxis(axis);
}

void kfcmd::core::Hypothesis::setAngularConstraintSigma(const std::string& constraintName,
                                                        double sigma) {
  auto cnt = dynamic_cast<nopt::NonLagrangeConstraint*>(_constraints.at(constraintName));
  // TODO: wrong class exception
  cnt->setLambda(1. / sigma / sigma);
}

void kfcmd::core::Hypothesis::disableVertexConstraintXYZ(
                                                         const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-x");
  disableConstraint("#" + chargedParticleName + "-constraint-y");
  disableConstraint("#" + chargedParticleName + "-constraint-z");
}

void kfcmd::core::Hypothesis::disableVertexConstraintX(
                                                       const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-x");
}

void kfcmd::core::Hypothesis::disableVertexConstraintY(
                                                       const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-y");
}

void kfcmd::core::Hypothesis::disableVertexConstraintZ(
                                                       const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-z");
}

void kfcmd::core::Hypothesis::enableVertexConstraintXYZ(
                                                        const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-x");
  enableConstraint("#" + chargedParticleName + "-constraint-y");
  enableConstraint("#" + chargedParticleName + "-constraint-z");
}

void kfcmd::core::Hypothesis::enableVertexConstraintX(
                                                      const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-x");
}

void kfcmd::core::Hypothesis::enableVertexConstraintY(
                                                      const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-y");
}

void kfcmd::core::Hypothesis::enableVertexConstraintZ(
                                                      const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-z");
}

TVector3 kfcmd::core::Hypothesis::getInitialVertex(const std::string& vertexName) const {
  return TVector3(getInitialCommonParameters("#" + vertexName + "-x")(0),
                  getInitialCommonParameters("#" + vertexName + "-y")(0),
                  getInitialCommonParameters("#" + vertexName + "-z")(0));
}

TVector3 kfcmd::core::Hypothesis::getFinalVertex(const std::string& vertexName) const {
  return TVector3(getFinalCommonParameters("#" + vertexName + "-x")(0),
                  getFinalCommonParameters("#" + vertexName + "-y")(0),
                  getFinalCommonParameters("#" + vertexName + "-z")(0));
}

double kfcmd::core::Hypothesis::getInitialVertexX(
                                                  const std::string& vertexName) const {
  return getInitialCommonParameters("#" + vertexName + "-x")(0);
}

double kfcmd::core::Hypothesis::getInitialVertexY(
                                                  const std::string& vertexName) const {
  return getInitialCommonParameters("#" + vertexName + "-y")(0);
}

double kfcmd::core::Hypothesis::getInitialVertexZ(
                                                  const std::string& vertexName) const {
  return getInitialCommonParameters("#" + vertexName + "-z")(0);
}

double kfcmd::core::Hypothesis::getFinalVertexX(const std::string& vertexName) const {
  return getFinalCommonParameters("#" + vertexName + "-x")(0);
}

double kfcmd::core::Hypothesis::getFinalVertexY(const std::string& vertexName) const {
  return getFinalCommonParameters("#" + vertexName + "-y")(0);
}

double kfcmd::core::Hypothesis::getFinalVertexZ(const std::string& vertexName) const {
  return getFinalCommonParameters("#" + vertexName + "-z")(0);
}

double kfcmd::core::Hypothesis::getInitialChargedParticleTime(const std::string& chargedParticleName) const {
  return getInitialParameters(chargedParticleName)(5); // !!! getInitialParameter(5)
}

double kfcmd::core::Hypothesis::getFinalChargedParticleTime(const std::string& chargedParticleName) const {
  return getFinalParameters(chargedParticleName)(5); // !!! getFinalParameter(5)
}

void kfcmd::core::Hypothesis::addMassConstraint(
                                                const std::string& constraintName, double mass,
                                                const std::set<std::string>& particleNames) {
  auto constraint = new kfbase::core::MassConstraint(constraintName, mass);
  addConstraint(constraint);
  for (const auto& name : particleNames) {
    addParticleToConstraint(name, constraintName);
  }
  enableConstraint(constraintName);
}

double kfcmd::core::Hypothesis::getEnergy() const { return _energy; }

TLorentzVector kfcmd::core::Hypothesis::getInitialRecoilMomentum(
                                                                 const std::set<std::string>& particleNames) const {
  TLorentzVector result(0, 0, 0, getEnergy());
  for (const auto& name : particleNames) {
    result -= getInitialMomentum(name);
  }
  return result;
}

TLorentzVector kfcmd::core::Hypothesis::getFinalRecoilMomentum(
                                                               const std::set<std::string>& particleNames) const {
  TLorentzVector result(0, 0, 0, getEnergy());
  for (const auto& name : particleNames) {
    result -= getFinalMomentum(name);
  }
  return result;
}

bool kfcmd::core::Hypothesis::checkMatrixInvertibility(
                                                       const Eigen::MatrixXd& matrix) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
  if (svd.singularValues()(svd.singularValues().size() - 1) == 0) return false;
  return true;
}

Eigen::MatrixXd kfcmd::core::Hypothesis::inverseMatrix(
                                                       const Eigen::MatrixXd& matrix) {
  return matrix.inverse();
}

bool kfcmd::core::Hypothesis::fillTrack(const std::string& name, std::size_t index,
                                        const kfcmd::core::TrPh& data) {
  Eigen::VectorXd par = Eigen::VectorXd::Zero(6);
  const Float_t* fst_terr0 = &((data.terr0)[index][0][0]);
  const int s = 5;
  const int sxs = s * s;
  double tmp_terr0[sxs];
  std::copy(fst_terr0, fst_terr0 + sxs, tmp_terr0);
  Eigen::Map<Eigen::Matrix<double, s, s, Eigen::RowMajor>> terr0(tmp_terr0);
  // indices in terr0:
  // 0 --- p
  // 1 --- phi
  // 2 --- rho
  // 3 --- ctg theta
  // 4 --- z
  Eigen::PermutationMatrix<s, s> perm;
  perm.indices() = {0, 2, 3, 1, 4};
  // Indices in kinfit are permutated. Indices in kinfit:
  // 0 --- p
  // 1 --- ctg
  // 2 --- phi
  // 3 --- rho
  // 4 --- z
  Eigen::MatrixXd cov = perm * terr0 * perm.inverse();
  cov.row(0) *= 1.e-3;
  cov.col(0) *= 1.e-3;
  if (0 == cov.determinant()) return false;
  par(0) = (data.tptot)[index] * std::sin((data.tth)[index]);
  par(0) *= 1.e-3;
  par(1) = 1. / std::tan((data.tth)[index]);
  par(2) = (data.tphi)[index];
  par(3) = (data.trho)[index];
  par(4) = (data.tz)[index];
  this->setInitialParticleParams(name, par);
  // index 6 --- ct
  Eigen::MatrixXd inv = Eigen::MatrixXd::Zero(6, 6);
  inv.block(0, 0, 5, 5) =  cov.inverse();
  this->setParticleInverseErrorMatrix(name, inv);
  return true;
}

bool kfcmd::core::Hypothesis::fillPhoton(const std::string& name, std::size_t index,
                                         const kfcmd::core::TrPh& data) {
  // 0 --- energy
  // 1 --- R
  // 2 --- phi
  // 3 --- z0
  Eigen::VectorXd par = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(4, 4);
  double sigma2_z = 0.30;
  double sigma2_rho = 0.35;
  double z = (data.phrho)[index] / tan((data.phth0)[index]);
  double sigma2_theta = (data.pherr)[index][1] * (data.pherr)[index][1];
  if (3 == (data.phflag)[index]) // checking BGO index
    sigma2_rho = sigma2_z * tan(data.phth0[index]) * tan(data.phth0[index]) +
      sigma2_theta * z * z / pow(cos(data.phth0[index]), 4);
  else
    sigma2_z = sigma2_rho / pow(tan((data.phth0)[index]), 2) +
      pow((data.phrho)[index] * (data.pherr)[index][1], 2) /
      pow(sin((data.phth0)[index]), 4);

  cov(0, 0) = pow((data.pherr)[index][0], 2);
  cov(1, 1) = sigma2_rho;
  cov(2, 2) = pow((data.pherr)[index][2], 2);
  cov(3, 3) = sigma2_z;
  cov.row(0) *= 1.e-3;
  cov.col(0) *= 1.e-3;
  if (0 == cov.determinant()) return false;
  par(0) = (data.phen)[index];
  par(0) *= 1.e-3;
  par(1) = (data.phrho)[index];
  par(2) = (data.phphi0)[index];
  par(3) = z;

  this->setInitialParticleParams(name, par);
  Eigen::MatrixXd inv = cov.inverse();
  this->setParticleInverseErrorMatrix(name, inv);
  return true;
}

void kfcmd::core::Hypothesis::setBeamXY(double xbeam, double ybeam) {
  addConstant("#beam-x", xbeam);
  addConstant("#beam-y", ybeam);
}
