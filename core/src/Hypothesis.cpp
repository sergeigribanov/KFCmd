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
#include "kfcmd/core/BGOLogNormalPhoton.hpp"
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

void kfcmd::core::Hypothesis::addBGOPhoton(const std::string& name,
                                           const std::string& vertexName) {
  auto particle = new kfcmd::core::BGOLogNormalPhoton(name);
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
  Eigen::VectorXd par = Eigen::VectorXd::Zero(7);
  const Float_t* fst_terr0 = &((data.terr0)[index][0][0]);
  const int s = 5;
  const int s_t = 6;
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
  double sigma2_z = 1.e-3;
  double sigma2_rho = 1.e-3;
  const double theta = (data.phth0)[index];
  const double rho = (data.phrad)[index] * sin(theta);
  double z = rho / tan(theta);
  double sigma2_theta = (data.pherr)[index][1] * (data.pherr)[index][1];
  if (3 == (data.phflag)[index]) // checking BGO index
    sigma2_rho = sigma2_z * tan(theta) * tan(theta) +
      sigma2_theta * z * z / pow(cos(theta), 4);
  else
    sigma2_z = sigma2_rho / pow(tan(theta), 2) +
      pow(rho * (data.pherr)[index][1], 2) /
      pow(sin(theta), 4);

  cov(0, 0) = pow((data.pherr)[index][0], 2);
  cov(1, 1) = sigma2_rho;
  cov(2, 2) = pow((data.pherr)[index][2], 2);
  cov(3, 3) = sigma2_z;
  cov.row(0) *= 1.e-3;
  cov.col(0) *= 1.e-3;
  if (0 == cov.determinant()) return false;
  par(0) = (data.phen)[index];
  par(0) *= 1.e-3;
  par(1) = rho;
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

// =============================================================================
// fillBSPhoton – strip version of Photon with beam-spot correction rollback
// =============================================================================
//
// Author: Dzmitry Shoukavy (shoukavy@ifanbel.bas-net.by)
// Algorithm description:
//   1. Input strip branches (bs_*): energy, angles "bs_phth0/bs_phphi0" are
//      corrected for the beam-spot position (xbeam, ybeam, z=0) as described
//      in Kuznetsov memo §3.10.  For kinematic fit, we must "return the angle
//      back to the detector centre" by reconstructing the true conversion point.
//   2. The ray from the anchor point (xbeam, ybeam, 0) along the direction
//      (theta, phi) is intersected with the cylinder of radius R = bs_phrho.
//      This yields the true conversion point (xc, yc, zc) via solving the
//      quadratic equation: a*t^2 + b*t + c = 0,
//        a = sin^2(theta)
//        b = 2*(xbeam*sin(theta)*cos(phi) + ybeam*sin(theta)*sin(phi))
//        c = xbeam^2 + ybeam^2 - R^2
//      The positive root t gives the point on the ray.
//   3. The covariance matrix is constructed following the same logic as in
//      the standard fillPhoton (tower), using bs_pherr.  For BGO (flag==3)
//      the z-coordinate of the cluster is well measured, so sigma_rho is
//      derived from sigma_z and sigma_theta; for LXe/CsI (barrel) the
//      opposite is true.
//   4. The parameters of the Photon particle are set to:
//        (E, R, phi_c, z_c)  where R = bs_phrho (cylindrical radius),
//        phi_c and z_c are the reconstructed conversion point coordinates.
//   5. The inverse covariance matrix is set for the fit.
//
bool kfcmd::core::Hypothesis::fillBSPhoton(const std::string& name,
                                           std::size_t index,
                                           const kfcmd::core::TrPh& data) {
  // Check index validity
  if (index >= (std::size_t)data.bs_nph) return false;

  // Extract strip parameters
  const double E_mev = (double)data.bs_phen[index];      // MeV
  const double theta = (double)data.bs_phth0[index];     // rad, beam-corrected polar angle
  const double phi   = (double)data.bs_phphi0[index];    // rad, beam-corrected azimuthal angle
  const double rho   = (double)data.bs_phrho[index];     // cm, cylindrical radius of conversion point
  const double st    = std::sin(theta);
  const double ct    = std::cos(theta);

  // Basic sanity checks
  if (!(E_mev > 0.) || !(rho > 1.) || !(st > 1.e-6)) return false;

  // ----- Rollback of beam-spot correction (Kuznetsov memo §3.10) -----
  const double xb = (double)data.xbeam;
  const double yb = (double)data.ybeam;

  const double nx = st * std::cos(phi);
  const double ny = st * std::sin(phi);
  const double nz = ct;

  const double a = nx*nx + ny*ny;   // = sin^2(theta)
  const double b = 2.0 * (xb*nx + yb*ny);
  const double c = xb*xb + yb*yb - rho*rho;

  const double disc = b*b - 4.0*a*c;
  if (disc <= 0.) return false;
  const double t = (-b + std::sqrt(disc)) / (2.0 * a);
  if (!(t > 0.)) return false;

  const double xc = xb + t * nx;
  const double yc = yb + t * ny;
  const double zc = t * nz;   // anchor z = 0

  // phi of conversion point in [0, 2pi)
  double phic = std::atan2(yc, xc);
  if (phic < 0.) phic += 2.0 * M_PI;

  // ----- Covariance matrix construction (following fillPhoton) -----
  const double sE  = (double)data.bs_pherr[index][0];   // MeV
  const double sTh = (double)data.bs_pherr[index][1];   // rad
  const double sPh = (double)data.bs_pherr[index][2];   // rad

  if (!(sE > 0.) || !(sTh > 0.) || !(sPh > 0.)) return false;

  double s2_rho = 1.e-3;   // cm^2 (seed)
  double s2_z   = 1.e-3;   // cm^2 (seed)

  if (data.bs_phflag[index] == 3) {
    // BGO (endcap): z is well measured, rho is derived
    s2_rho = s2_z * std::tan(theta) * std::tan(theta) +
      sTh * sTh * zc * zc / std::pow(std::cos(theta), 4);
  } else {
    // LXe/CsI (barrel): rho is well measured, z is derived
    s2_z = s2_rho / (std::tan(theta) * std::tan(theta)) +
      std::pow(rho * sTh, 2) / std::pow(st, 4);
  }

  Eigen::VectorXd par(4);
  par << E_mev * 1.e-3,   // energy in GeV
    rho,            // R_c (cylindrical radius)
    phic,           // phi_c
    zc;             // z_c

  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(4, 4);
  cov(0, 0) = std::pow(sE * 1.e-3, 2);
  cov(1, 1) = s2_rho;
  cov(2, 2) = sPh * sPh;
  cov(3, 3) = s2_z;

  if (cov.determinant() == 0.) return false;

  this->setInitialParticleParams(name, par);
  this->setParticleInverseCovarianceMatrix(name, cov.inverse());
  return true;
}

// ----------------------------------------------------------------------
// fillBGOPhoton – strip version of Photon with log‑normal energy PDF
// ----------------------------------------------------------------------
bool kfcmd::core::Hypothesis::fillBGOPhoton(const std::string& name,
                                            std::size_t index,
                                            const kfcmd::core::TrPh& data) {
  // Basic checks and index validity
  if (index >= (std::size_t)data.bs_nph) return false;

  const double E_mev = (double)data.bs_phen[index];
  const double theta = (double)data.bs_phth0[index];
  const double phi   = (double)data.bs_phphi0[index];
  const double rho   = (double)data.bs_phrho[index];
  const double st    = std::sin(theta);
  if (!(E_mev > 0.) || !(rho > 1.) || !(st > 1.e-6)) return false;

  // ----- Rollback of beam-spot correction (Kuznetsov memo §3.10) -----
  const double xb = (double)data.xbeam;
  const double yb = (double)data.ybeam;
  const double nx = st * std::cos(phi);
  const double ny = st * std::sin(phi);
  const double nz = std::cos(theta);
  const double a  = nx*nx + ny*ny;
  const double b  = 2.0 * (xb*nx + yb*ny);
  const double c  = xb*xb + yb*yb - rho*rho;
  const double disc = b*b - 4.0*a*c;
  if (disc <= 0.) return false;
  const double t = (-b + std::sqrt(disc)) / (2.0 * a);
  if (!(t > 0.)) return false;
  const double xc = xb + t * nx;
  const double yc = yb + t * ny;
  const double zc = t * nz;
  double phic = std::atan2(yc, xc);
  if (phic < 0.) phic += 2.0 * M_PI;

  // ----- Covariance matrix construction (same as fillPhoton) -----
  const double sE  = (double)data.bs_pherr[index][0];
  const double sTh = (double)data.bs_pherr[index][1];
  const double sPh = (double)data.bs_pherr[index][2];
  if (!(sE > 0.) || !(sTh > 0.) || !(sPh > 0.)) return false;

  double s2_rho = 1.e-3;
  double s2_z   = 1.e-3;
  if (data.bs_phflag[index] == 3) {
    // BGO (endcap): z is well measured, rho is derived
    s2_rho = s2_z * std::tan(theta) * std::tan(theta) +
      sTh * sTh * zc * zc / std::pow(std::cos(theta), 4);
  } else {
    // LXe/CsI (barrel): rho is well measured, z is derived
    s2_z = s2_rho / (std::tan(theta) * std::tan(theta)) +
      std::pow(rho * sTh, 2) / std::pow(st, 4);
  }

  Eigen::VectorXd par(4);
  par << E_mev * 1.e-3, rho, phic, zc;

  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(4, 4);
  cov(0, 0) = std::pow(sE * 1.e-3, 2);
  cov(1, 1) = s2_rho;
  cov(2, 2) = sPh * sPh;
  cov(3, 3) = s2_z;
  if (cov.determinant() == 0.) return false;

  // Get the particle and set its parameters directly
  auto* particle = getParticle(name);
  if (!particle) return false;

  auto* bgo = dynamic_cast<kfcmd::core::BGOLogNormalPhoton*>(particle);
  if (!bgo) return false;

  bgo->setInitialParameters(par);

  Eigen::MatrixXd inv = cov.inverse();
  inv(0, 0) = 0.0; // remove Gaussian penalty on energy
  bgo->setInverseCovarianceMatrix(inv);

  // Set measured energy, endcap, season and beam energy
  bgo->setMeasuredEnergy(E_mev * 1.e-3);
  bgo->setEndcap( (theta < M_PI/2) ? 0 : 1 );
  bgo->setSeason(season_);

  return true;
}

bool kfcmd::core::Hypothesis::fillAltBSPhoton(const std::string& name,
					      std::size_t index,
					      const kfcmd::core::TrPh& data) {
  Eigen::VectorXd par(3);
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(3, 3);
  par(0) = (data.bs_phen)[index] * 1.e-3;
  par(1) = (data.bs_phth)[index];
  par(2) = (data.bs_phphi)[index];
  cov(0, 0) = std::pow((data.bs_pherr)[index][0] * 1.e-3, 2);
  cov(1, 1) = std::pow((data.bs_pherr)[index][1], 2);
  cov(2, 2) = std::pow((data.bs_pherr)[index][2], 2);
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

void kfcmd::core::Hypothesis::setSeason(const std::string& s) {
  season_ = s;
}
