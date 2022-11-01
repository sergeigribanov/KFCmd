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
#include "kfcmd/core/ParticlePxPyPz.hpp"
#include "kfcmd/core/ParticleMassLessThetaPhiE.hpp"
#include "kfcmd/core/AltPhoton.hpp"
#include <kfbase/core/MassConstraint.hpp>
#include <kfbase/core/MomentumConstraint.hpp>
#include <kfbase/core/DoubleParticleAngularConstraint.hpp>
#include <kfbase/core/ParticleAngularConstraint.hpp>
#include <kfbase/core/IntermediateNeutralParticle.hpp>
#include <kfbase/core/VertexXYZ.hpp>
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

void kfcmd::core::Hypothesis::addVertexXYZ(const std::string& vertexName) {
  if (vertices_.find(vertexName) != vertices_.end()) {
    // TO DO : exception;
    return;
  }
  auto vtx = new kfbase::core::VertexXYZ(vertexName);
  vtx->setLowerLimit(0, -30);
  vtx->setUpperLimit(0, 30);
  vtx->setLowerLimit(1, -30);
  vtx->setUpperLimit(1, 30);
  vtx->setLowerLimit(2, -20);
  vtx->setUpperLimit(2, 20);
  addVertex(vtx);
}

void kfcmd::core::Hypothesis::addChargedParticle(kfcmd::core::ChargedParticle* particle) {
  addParticle(particle);
  particle->setMagneticField("#m-field");
  particle->setBeamX("#beam-x");
  particle->setBeamY("#beam-y");
}

void kfcmd::core::Hypothesis::addPhoton(const std::string& name,
                                        const std::string& vertexName) {
  auto particle = new kfcmd::core::Photon(name);
  addParticle(particle);
  auto vtx = vertices_.at(vertexName);
  particle->setOutputVertex(vtx);
}

void kfcmd::core::Hypothesis::addAltPhoton(const std::string& name) {
  auto particle = new kfcmd::core::AltPhoton(name);
  addParticle(particle);
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
  auto vtx = vertices_.at(vertexName); // !!! TODO: exception
  particle->setOutputVertex(vtx);
}

void kfcmd::core::Hypothesis::addParticlePxPyPz(const std::string& name, double mass) {
  auto particle = new kfcmd::core::ParticlePxPyPz(name, mass);
  addParticle(particle);
}

void kfcmd::core::Hypothesis::addParticleMassLessThetaPhiE(const std::string& name) {
  auto particle = new kfcmd::core::ParticleMassLessThetaPhiE(name);
  addParticle(particle);
}

void kfcmd::core::Hypothesis::addOutputVertexConstraintsXYZ(const std::string& vertexParticleName,
                                                            const std::string& vertexName) {
  auto vtx = vertices_.at(vertexName); // !!! TODO: exception
  auto vtxX = new kfbase::core::OutputVertexConstraint("#" + vertexParticleName +
                                                       "-output-constraint-x", kfbase::core::VERTEX_X);
  addConstraint(vtxX);
  vtxX->setVertex(vtx);

  auto vtxY = new kfbase::core::OutputVertexConstraint("#" + vertexParticleName +
                                                       "-output-constraint-y", kfbase::core::VERTEX_Y);
  addConstraint(vtxY);
  vtxY->setVertex(vtx);

  auto vtxZ = new kfbase::core::OutputVertexConstraint("#" + vertexParticleName +
                                                       "-output-constraint-z", kfbase::core::VERTEX_Z);
  addConstraint(vtxZ);
  vtxZ->setVertex(vtx);

  addParticleToConstraint(vertexParticleName, vtxX->getName());
  addParticleToConstraint(vertexParticleName, vtxY->getName());
  addParticleToConstraint(vertexParticleName, vtxZ->getName());

  enableConstraint(vtxX->getName());
  enableConstraint(vtxY->getName());
  enableConstraint(vtxZ->getName());
}

void kfcmd::core::Hypothesis::addInputVertexConstraintsXYZ(const std::string& vertexParticleName,
                                                            const std::string& vertexName) {
  auto vtx = vertices_.at(vertexName); // !!! TODO: exception
  const auto& particle = dynamic_cast<kfcmd::core::ChargedParticle*>(_particles.at(vertexParticleName));
  if (particle) {
    particle->releaseParameter(6);
  }

  auto vtxX = new kfbase::core::InputVertexConstraint("#" + vertexParticleName +
                                                      "-input-constraint-x", kfbase::core::VERTEX_X);
  addConstraint(vtxX);
  vtxX->setVertex(vtx);

  auto vtxY = new kfbase::core::InputVertexConstraint("#" + vertexParticleName +
                                                       "-input-constraint-y", kfbase::core::VERTEX_Y);
  addConstraint(vtxY);
  vtxY->setVertex(vtx);

  auto vtxZ = new kfbase::core::InputVertexConstraint("#" + vertexParticleName +
                                                       "-input-constraint-z", kfbase::core::VERTEX_Z);
  addConstraint(vtxZ);
  vtxZ->setVertex(vtx);

  addParticleToConstraint(vertexParticleName, vtxX->getName());
  addParticleToConstraint(vertexParticleName, vtxY->getName());
  addParticleToConstraint(vertexParticleName, vtxZ->getName());

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

void kfcmd::core::Hypothesis::disableOutputVertexConstraintXYZ(const std::string& vertexParticleName) {
  disableOutputVertexConstraintX(vertexParticleName);
  disableOutputVertexConstraintY(vertexParticleName);
  disableOutputVertexConstraintZ(vertexParticleName);
}

void kfcmd::core::Hypothesis::disableInputVertexConstraintXYZ(const std::string& vertexParticleName) {
  disableInputVertexConstraintX(vertexParticleName);
  disableInputVertexConstraintY(vertexParticleName);
  disableInputVertexConstraintZ(vertexParticleName);
}

void kfcmd::core::Hypothesis::disableOutputVertexConstraintX(const std::string& vertexParticleName) {
  disableConstraint("#" + vertexParticleName + "-output-constraint-x");
}

void kfcmd::core::Hypothesis::disableInputVertexConstraintX(const std::string& vertexParticleName) {
  disableConstraint("#" + vertexParticleName + "-input-constraint-x");
}

void kfcmd::core::Hypothesis::disableOutputVertexConstraintY(const std::string& vertexParticleName) {
  disableConstraint("#" + vertexParticleName + "-output-constraint-y");
}

void kfcmd::core::Hypothesis::disableInputVertexConstraintY(const std::string& vertexParticleName) {
  disableConstraint("#" + vertexParticleName + "-input-constraint-y");
}

void kfcmd::core::Hypothesis::disableOutputVertexConstraintZ(const std::string& vertexParticleName) {
  disableConstraint("#" + vertexParticleName + "-output-constraint-z");
}

void kfcmd::core::Hypothesis::disableInputVertexConstraintZ(const std::string& vertexParticleName) {
  disableConstraint("#" + vertexParticleName + "-input-constraint-z");
}

void kfcmd::core::Hypothesis::enableOutputVertexConstraintXYZ(const std::string& vertexParticleName) {
  enableOutputVertexConstraintX(vertexParticleName);
  enableOutputVertexConstraintY(vertexParticleName);
  enableOutputVertexConstraintZ(vertexParticleName);
}

void kfcmd::core::Hypothesis::enableInputVertexConstraintXYZ(const std::string& vertexParticleName) {
  enableInputVertexConstraintX(vertexParticleName);
  enableInputVertexConstraintY(vertexParticleName);
  enableInputVertexConstraintZ(vertexParticleName);
}

void kfcmd::core::Hypothesis::enableOutputVertexConstraintX(const std::string& vertexParticleName) {
  enableConstraint("#" + vertexParticleName + "-output-constraint-x");
}

void kfcmd::core::Hypothesis::enableInputVertexConstraintX(const std::string& vertexParticleName) {
  enableConstraint("#" + vertexParticleName + "-input-constraint-x");
}

void kfcmd::core::Hypothesis::enableOutputVertexConstraintY(const std::string& vertexParticleName) {
  enableConstraint("#" + vertexParticleName + "-output-constraint-y");
}

void kfcmd::core::Hypothesis::enableInputVertexConstraintY(const std::string& vertexParticleName) {
  enableConstraint("#" + vertexParticleName + "-input-constraint-y");
}

void kfcmd::core::Hypothesis::enableOutputVertexConstraintZ(const std::string& vertexParticleName) {
  enableConstraint("#" + vertexParticleName + "-output-constraint-z");
}

void kfcmd::core::Hypothesis::enableInputVertexConstraintZ(const std::string& vertexParticleName) {
  enableConstraint("#" + vertexParticleName + "-input-constraint-z");
}

TVector3 kfcmd::core::Hypothesis::getInitialVertex(const std::string& name) const {
  return vertices_.at(name)->getInitialXYZ();
}

TVector3 kfcmd::core::Hypothesis::getFinalVertex(const std::string& name) const {
  return vertices_.at(name)->getFinalXYZ();
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
  Eigen::VectorXd par = Eigen::VectorXd::Zero(6); // 7
  const Float_t* fst_terr0 = &((data.terr0)[index][0][0]);
  const int s = 5;
  const int s_t = 5; // 6
  const int sxs_t = s_t * s_t;
  double tmp_terr0[sxs_t];
  std::copy(fst_terr0, fst_terr0 + sxs_t, tmp_terr0);
  Eigen::Map<Eigen::Matrix<double, s_t, s_t, Eigen::RowMajor>> terr0(tmp_terr0);
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
  Eigen::MatrixXd cov = perm * terr0.block(0, 0, 5, 5) * perm.inverse();
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
  // index 5 --- ct_out
  // index 6 --- ct_in
  Eigen::MatrixXd inv = Eigen::MatrixXd::Zero(7, 7);
  inv.block(0, 0, 5, 5) =  cov.inverse();
  this->setParticleInverseCovarianceMatrix(name, inv);
  return true;
}


bool kfcmd::core::Hypothesis::fillPhoton(const std::string& name,
                                         std::size_t index,
                                         const kfcmd::core::TrPh& data) {
  // 0 --- energy
  // 1 --- rho
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
  this->setParticleInverseCovarianceMatrix(name, inv);
  return true;
}

bool kfcmd::core::Hypothesis::fillAltPhoton(const std::string& name,
                                            std::size_t index,
                                            const kfcmd::core::TrPh& data) {
  Eigen::VectorXd par(3);
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(3, 3);
  par(0) = (data.phen)[index] * 1.e-3;
  par(1) = (data.phth)[index];
  par(2) = (data.phphi)[index];
  cov(0, 0) = std::pow((data.pherr)[index][0] * 1.e-3, 2);
  cov(1, 1) = std::pow((data.pherr)[index][1], 2);
  cov(2, 2) = std::pow((data.pherr)[index][2], 2);
  if (0 == cov.determinant()) return false;
  this->setInitialParticleParams(name, par);
  Eigen::MatrixXd inv = cov.inverse();
  this->setParticleInverseCovarianceMatrix(name, inv);
  return true;
}

void kfcmd::core::Hypothesis::setBeamXY(double xbeam, double ybeam) {
  addConstant("#beam-x", xbeam);
  addConstant("#beam-y", ybeam);
}
