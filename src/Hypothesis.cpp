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

#include "Hypothesis.hpp"

#include <KFBase/MassConstraint.hpp>
#include <KFBase/MomentumConstraint.hpp>
#include <KFBase/FlowConstraint.hpp>
#include <ccgo/CommonParams.hpp>
#include <cmath>

KFCmd::Hypothesis::Hypothesis(double energy, double magneticField, long nIter,
                              double tolerance)
    : KFBase::Hypothesis(nIter, tolerance), _energy(energy) {
  addConstant("#m-field", magneticField);
  addConstraint(
      new KFBase::MomentumConstraint("#constraint-px", KFBase::MOMENT_X, 0));
  addConstraint(
      new KFBase::MomentumConstraint("#constraint-py", KFBase::MOMENT_Y, 0));
  addConstraint(
      new KFBase::MomentumConstraint("#constraint-pz", KFBase::MOMENT_Z, 0));
  addConstraint(new KFBase::MomentumConstraint("#constraint-pe",
                                               KFBase::MOMENT_E, energy));
  enableConstraint("#constraint-px");
  enableConstraint("#constraint-py");
  enableConstraint("#constraint-pz");
  enableConstraint("#constraint-pe");
}

KFCmd::Hypothesis::~Hypothesis() {}

void KFCmd::Hypothesis::enableCommonMomentumConstraintPxPyPzE() {
  enableConstraint("#constraint-px");
  enableConstraint("#constraint-py");
  enableConstraint("#constraint-pz");
  enableConstraint("#constraint-pe");
}

void KFCmd::Hypothesis::disableCommonMomentumConstraintPxPyPzE() {
  disableConstraint("#constraint-px");
  disableConstraint("#constraint-py");
  disableConstraint("#constraint-pz");
  disableConstraint("#constraint-pe");
}

void KFCmd::Hypothesis::setInitialVertex(const std::string& vertexName,
					 const Eigen::Vector3d& vertex) {
  const std::string vx = "#" + vertexName + "-x";
  const std::string vy = "#" + vertexName + "-y";
  const std::string vz = "#" + vertexName + "-z";
  Eigen::VectorXd xvect(1);
  xvect(0) = vertex(0);
  Eigen::VectorXd yvect(1);
  yvect(0) = vertex(1);
  Eigen::VectorXd zvect(1);
  zvect(0) = vertex(2);
  setInitialCommonParams(vx, xvect);
  setInitialCommonParams(vy, yvect);
  setInitialCommonParams(vz, zvect);
}

void KFCmd::Hypothesis::setInitialVertexRandomly(const std::string& vertexName, double distance) {
  Eigen::Vector3d vertex = distance * Eigen::Vector3d::Random();
  setInitialVertex(vertexName, vertex);
}

void KFCmd::Hypothesis::addVertex(const std::string& vertexName) {
  if (_vertices.find(vertexName) != _vertices.end()) {
    // TO DO : exception;
    return;
  }
  const std::string vx = "#" + vertexName + "-x";
  const std::string vy = "#" + vertexName + "-y";
  const std::string vz = "#" + vertexName + "-z";
  auto xvertex = new ccgo::CommonParams(vx, 1);
  xvertex->setLowerLimit(0, -30);
  xvertex->setUpperLimit(0, 30);
  auto yvertex = new ccgo::CommonParams(vy, 1);
  yvertex->setLowerLimit(0, -30);
  yvertex->setUpperLimit(0, 30);
  auto zvertex = new ccgo::CommonParams(vz, 1);
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

void KFCmd::Hypothesis::disableVertex(const std::string& vertexName) {
  disableCommonParams("#" + vertexName + "-x");
  disableCommonParams("#" + vertexName + "-y");
  disableCommonParams("#" + vertexName + "-z");
}

void KFCmd::Hypothesis::enableVertex(const std::string& vertexName) {
  enableCommonParams("#" + vertexName + "-x");
  enableCommonParams("#" + vertexName + "-y");
  enableCommonParams("#" + vertexName + "-z");
}

void KFCmd::Hypothesis::disableVertexComponent(
    const std::string& vertexName, KFBase::VERTEX_COMPONENT component) {
  switch (component) {
    case KFBase::VERTEX_X:
      disableCommonParams("#" + vertexName + "-x");
      break;
    case KFBase::VERTEX_Y:
      disableCommonParams("#" + vertexName + "-y");
      break;
    case KFBase::VERTEX_Z:
      disableCommonParams("#" + vertexName + "-z");
      break;
  }
}

void KFCmd::Hypothesis::enableVertexComponent(
    const std::string& vertexName, KFBase::VERTEX_COMPONENT component) {
  switch (component) {
    case KFBase::VERTEX_X:
      enableCommonParams("#" + vertexName + "-x");
      break;
    case KFBase::VERTEX_Y:
      enableCommonParams("#" + vertexName + "-y");
      break;
    case KFBase::VERTEX_Z:
      enableCommonParams("#" + vertexName + "-z");
      break;
  }
}

TVector3 KFCmd::Hypothesis::calcVertexComponent(const std::string& chargedParticleName) {
  const auto& particle = dynamic_cast<KFCmd::ChargedParticle*>(_particles.at(chargedParticleName));
  Eigen::VectorXd x(_opt.getN());
  x.segment(particle->getBeginIndex(), particle->getN()) = particle->getBeginParameters();
  const std::string timeParameter = "#time-" + particle->getName();
  const auto& cp = dynamic_cast<ccgo::CommonParams*>(_commonParams.at(timeParameter));
  x.segment(cp->getBeginIndex(), cp->getN()) = cp->getBeginParameters();
  TVector3 result;
  result(0) = particle->calcVertexComponent(x, KFBase::VERTEX_X);
  result(1) = particle->calcVertexComponent(x, KFBase::VERTEX_Y);
  result(2) = particle->calcVertexComponent(x, KFBase::VERTEX_Z);
  return result;
}

void KFCmd::Hypothesis::fixVertexComponent(const std::string& vertexName,
                                           double value,
                                           KFBase::VERTEX_COMPONENT component) {
  switch (component) {
    case KFBase::VERTEX_X:
      _commonParams.at("#" + vertexName + "-x")->fixParameter(0, value);
      break;
    case KFBase::VERTEX_Y:
      _commonParams.at("#" + vertexName + "-y")->fixParameter(0, value);
      break;
    case KFBase::VERTEX_Z:
      _commonParams.at("#" + vertexName + "-z")->fixParameter(0, value);
      break;
  }
}

void KFCmd::Hypothesis::releaseVertexComponent(
    const std::string& vertexName, KFBase::VERTEX_COMPONENT component) {
  switch (component) {
    case KFBase::VERTEX_X:
      _commonParams.at("#" + vertexName + "-x")->releaseParameter(0);
      break;
    case KFBase::VERTEX_Y:
      _commonParams.at("#" + vertexName + "-y")->releaseParameter(0);
      break;
    case KFBase::VERTEX_Z:
      _commonParams.at("#" + vertexName + "-z")->releaseParameter(0);
      break;
  }
}

void KFCmd::Hypothesis::fixTrackNaturalParameter(const std::string& charged_particle_name,
						 double value) {
  _commonParams.at("#time-" + charged_particle_name)->fixParameter(0, value);
}

void KFCmd::Hypothesis::releaseTrackNaturalParameter(const std::string& charged_particle_name) {
  _commonParams.at("#time-" + charged_particle_name)->releaseParameter(0);
}

void KFCmd::Hypothesis::addChargedParticle(KFCmd::ChargedParticle* particle) {
  addParticle(particle);
  particle->setMagneticField("#m-field");
  const std::string timeParameter = "#time-" + particle->getName();
  auto timep = new ccgo::CommonParams(timeParameter, 1);
  timep->setLowerLimit(0, -100);
  timep->setUpperLimit(0, 100);
  addCommonParams(timep);
  particle->setTimeParameter(timeParameter);
  enableParticle(particle->getName());
  enableCommonParams(timeParameter);
  addParticleToConstraint(particle->getName(), "#constraint-px");
  addParticleToConstraint(particle->getName(), "#constraint-py");
  addParticleToConstraint(particle->getName(), "#constraint-pz");
  addParticleToConstraint(particle->getName(), "#constraint-pe");
}

void KFCmd::Hypothesis::disableChargedParticle(
    const std::string& chargedParticleName) {
  disableParticle(chargedParticleName);
  disableCommonParams("#time-" + chargedParticleName);
}

void KFCmd::Hypothesis::enableChargedParticle(
    const std::string& chargedParticleName) {
  enableParticle(chargedParticleName);
  enableCommonParams("#time-" + chargedParticleName);
}

void KFCmd::Hypothesis::addPhoton(KFCmd::Photon* photon,
                                  const std::string& vertexName) {
  addParticle(photon);
  const std::string vertexXname = "#" + vertexName + "-x";
  photon->setVertexX(vertexXname);
  const std::string vertexYname = "#" + vertexName + "-y";
  photon->setVertexY(vertexYname);
  const std::string vertexZname = "#" + vertexName + "-z";
  photon->setVertexZ(vertexZname);
  enableParticle(photon->getName());
  addParticleToConstraint(photon->getName(), "#constraint-px");
  addParticleToConstraint(photon->getName(), "#constraint-py");
  addParticleToConstraint(photon->getName(), "#constraint-pz");
  addParticleToConstraint(photon->getName(), "#constraint-pe");
  _constraints.at("#constraint-px")->includeUsedCommonParameter(vertexXname);
  _constraints.at("#constraint-px")->includeUsedCommonParameter(vertexYname);
  _constraints.at("#constraint-px")->includeUsedCommonParameter(vertexZname);
  _constraints.at("#constraint-py")->includeUsedCommonParameter(vertexXname);
  _constraints.at("#constraint-py")->includeUsedCommonParameter(vertexYname);
  _constraints.at("#constraint-py")->includeUsedCommonParameter(vertexZname);
  _constraints.at("#constraint-pz")->includeUsedCommonParameter(vertexXname);
  _constraints.at("#constraint-pz")->includeUsedCommonParameter(vertexYname);
  _constraints.at("#constraint-pz")->includeUsedCommonParameter(vertexZname);
}

void KFCmd::Hypothesis::disablePhoton(const std::string& photonName) {
  disableParticle(photonName);
}

void KFCmd::Hypothesis::enablePhoton(const std::string& photonName) {
  enableParticle(photonName);
}

void KFCmd::Hypothesis::addVertexConstraintsXYZ(
    const std::string& chargedParticleName, const std::string& vertexName) {
  auto vtxX = new KFBase::VertexConstraint(
      "#" + chargedParticleName + "-constraint-x", KFBase::VERTEX_X);
  addConstraint(vtxX);
  const std::string vertexXname = "#" + vertexName + "-x";
  vtxX->setVertexCommonParams(vertexXname);
  auto vtxY = new KFBase::VertexConstraint(
      "#" + chargedParticleName + "-constraint-y", KFBase::VERTEX_Y);
  addConstraint(vtxY);
  const std::string vertexYname = "#" + vertexName + "-y";
  vtxY->setVertexCommonParams(vertexYname);
  auto vtxZ = new KFBase::VertexConstraint(
      "#" + chargedParticleName + "-constraint-z", KFBase::VERTEX_Z);
  addConstraint(vtxZ);
  const std::string vertexZname = "#" + vertexName + "-z";
  vtxZ->setVertexCommonParams(vertexZname);
  addParticleToConstraint(chargedParticleName, vtxX->getName());
  addParticleToConstraint(chargedParticleName, vtxY->getName());
  addParticleToConstraint(chargedParticleName, vtxZ->getName());
  const std::string timeParameter = "#time-" + chargedParticleName;
  _constraints.at(vtxX->getName())->includeUsedCommonParameter(timeParameter);
  _constraints.at(vtxY->getName())->includeUsedCommonParameter(timeParameter);
  _constraints.at(vtxZ->getName())->includeUsedCommonParameter(timeParameter);
  _constraints.at(vtxX->getName())->includeUsedCommonParameter(vertexXname);
  _constraints.at(vtxY->getName())->includeUsedCommonParameter(vertexYname);
  _constraints.at(vtxZ->getName())->includeUsedCommonParameter(vertexZname);
  enableConstraint(vtxX->getName());
  enableConstraint(vtxY->getName());
  enableConstraint(vtxZ->getName());
}

void KFCmd::Hypothesis::addFlowConstraintsXYZ(const std::string& flowName,
					      const std::string& beginVertexName,
					      const std::string& endVertexName) {
  auto flowX = new KFBase::FlowConstraint("#" + flowName + "-x", KFBase::FLOW_X);
  auto flowY = new KFBase::FlowConstraint("#" + flowName + "-y", KFBase::FLOW_Y);
  auto flowZ = new KFBase::FlowConstraint("#" + flowName + "-z", KFBase::FLOW_Z);
  addConstraint(flowX);
  addConstraint(flowY);
  addConstraint(flowZ);
  const std::string bvX = "#" + beginVertexName + "-x";
  const std::string bvY = "#" + beginVertexName + "-y";
  const std::string bvZ = "#" + beginVertexName + "-z";
  const std::string evX = "#" + endVertexName + "-x";
  const std::string evY = "#" + endVertexName + "-y";
  const std::string evZ = "#" + endVertexName + "-z";
  flowX->setBeginVertexCommonParams(bvX, bvY, bvZ);
  flowX->setEndVertexCommonParams(evX, evY, evZ);
  flowY->setBeginVertexCommonParams(bvX, bvY, bvZ);
  flowY->setEndVertexCommonParams(evX, evY, evZ);
  flowZ->setBeginVertexCommonParams(bvX, bvY, bvZ);
  flowZ->setEndVertexCommonParams(evX, evY, evZ);
  flowX->includeUsedCommonParameter(bvX);
  flowX->includeUsedCommonParameter(bvY);
  flowX->includeUsedCommonParameter(bvZ);
  flowX->includeUsedCommonParameter(evX);
  flowX->includeUsedCommonParameter(evY);
  flowX->includeUsedCommonParameter(evZ);
  flowY->includeUsedCommonParameter(bvX);
  flowY->includeUsedCommonParameter(bvY);
  flowY->includeUsedCommonParameter(bvZ);
  flowY->includeUsedCommonParameter(evX);
  flowY->includeUsedCommonParameter(evY);
  flowY->includeUsedCommonParameter(evZ);
  flowZ->includeUsedCommonParameter(bvX);
  flowZ->includeUsedCommonParameter(bvY);
  flowZ->includeUsedCommonParameter(bvZ);
  flowZ->includeUsedCommonParameter(evX);
  flowZ->includeUsedCommonParameter(evY);
  flowZ->includeUsedCommonParameter(evZ);
  enableConstraint(flowX->getName());
  enableConstraint(flowY->getName());
  enableConstraint(flowZ->getName());
}

void KFCmd::Hypothesis::addParticleToFlow(const std::string& flowName,
					  const std::string& particleName) {
  addParticleToConstraint(particleName, "#" + flowName + "-x");
  addParticleToConstraint(particleName, "#" + flowName + "-y");
  addParticleToConstraint(particleName, "#" + flowName + "-z");
  const auto& particle = dynamic_cast<KFCmd::ChargedParticle*>(_particles.at(particleName));
  if (particle) {
    const std::string timeParameter = "#time-" + particleName;
    _constraints.at("#" + flowName + "-x")->includeUsedCommonParameter(timeParameter);
    _constraints.at("#" + flowName + "-y")->includeUsedCommonParameter(timeParameter);
    _constraints.at("#" + flowName + "-z")->includeUsedCommonParameter(timeParameter);
  }
}

void KFCmd::Hypothesis::disableFlowConstraintXYZ(const std::string& flowName) {
  disableConstraint("#" + flowName + "-x");
  disableConstraint("#" + flowName + "-y");
  disableConstraint("#" + flowName + "-z");
}

void KFCmd::Hypothesis::enableFlowConstraintXYZ(const std::string& flowName) {
  enableConstraint("#" + flowName + "-x");
  enableConstraint("#" + flowName + "-y");
  enableConstraint("#" + flowName + "-z");
}

void KFCmd::Hypothesis::disableVertexConstraintXYZ(
    const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-x");
  disableConstraint("#" + chargedParticleName + "-constraint-y");
  disableConstraint("#" + chargedParticleName + "-constraint-z");
  disableCommonParams("#time-" + chargedParticleName);
}

void KFCmd::Hypothesis::disableVertexConstraintX(
    const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-x");
  bool flag =
      isConstraintEnabled("#" + chargedParticleName + "-constraint-y") ||
      isConstraintEnabled("#" + chargedParticleName + "-constraint-z");
  if (!flag) {
    disableCommonParams("#time-" + chargedParticleName);
  }
}

void KFCmd::Hypothesis::disableVertexConstraintY(
    const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-y");
  bool flag =
      isConstraintEnabled("#" + chargedParticleName + "-constraint-x") ||
      isConstraintEnabled("#" + chargedParticleName + "-constraint-z");
  if (!flag) {
    disableCommonParams("#time-" + chargedParticleName);
  }
}

void KFCmd::Hypothesis::disableVertexConstraintZ(
    const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-z");
  bool flag =
      isConstraintEnabled("#" + chargedParticleName + "-constraint-x") ||
      isConstraintEnabled("#" + chargedParticleName + "-constraint-y");
  if (!flag) {
    disableCommonParams("#time-" + chargedParticleName);
  }
}

void KFCmd::Hypothesis::enableVertexConstraintXYZ(
    const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-x");
  enableConstraint("#" + chargedParticleName + "-constraint-y");
  enableConstraint("#" + chargedParticleName + "-constraint-z");
  enableCommonParams("#time-" + chargedParticleName);
}

void KFCmd::Hypothesis::enableVertexConstraintX(
    const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-x");
  enableCommonParams("#time-" + chargedParticleName);
}

void KFCmd::Hypothesis::enableVertexConstraintY(
    const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-y");
  enableCommonParams("#time-" + chargedParticleName);
}

void KFCmd::Hypothesis::enableVertexConstraintZ(
    const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-z");
  enableCommonParams("#time-" + chargedParticleName);
}

TVector3 KFCmd::Hypothesis::getInitialVertex(const std::string& vertexName) const {
  return TVector3(getInitialCommonParameters("#" + vertexName + "-x")(0),
		  getInitialCommonParameters("#" + vertexName + "-y")(0),
		  getInitialCommonParameters("#" + vertexName + "-z")(0));
}

TVector3 KFCmd::Hypothesis::getFinalVertex(const std::string& vertexName) const {
  return TVector3(getFinalCommonParameters("#" + vertexName + "-x")(0),
		  getFinalCommonParameters("#" + vertexName + "-y")(0),
		  getFinalCommonParameters("#" + vertexName + "-z")(0));
}

double KFCmd::Hypothesis::getInitialVertexX(
    const std::string& vertexName) const {
  return getInitialCommonParameters("#" + vertexName + "-x")(0);
}

double KFCmd::Hypothesis::getInitialVertexY(
    const std::string& vertexName) const {
  return getInitialCommonParameters("#" + vertexName + "-y")(0);
}

double KFCmd::Hypothesis::getInitialVertexZ(
    const std::string& vertexName) const {
  return getInitialCommonParameters("#" + vertexName + "-z")(0);
}

double KFCmd::Hypothesis::getFinalVertexX(const std::string& vertexName) const {
  return getFinalCommonParameters("#" + vertexName + "-x")(0);
}

double KFCmd::Hypothesis::getFinalVertexY(const std::string& vertexName) const {
  return getFinalCommonParameters("#" + vertexName + "-y")(0);
}

double KFCmd::Hypothesis::getFinalVertexZ(const std::string& vertexName) const {
  return getFinalCommonParameters("#" + vertexName + "-z")(0);
}

double KFCmd::Hypothesis::getInitialChargedParticleTime(
    const std::string& chargedParticleName) const {
  return getInitialCommonParameters("#time-" + chargedParticleName)(0);
}

double KFCmd::Hypothesis::getFinalChargedParticleTime(
    const std::string& chargedParticleName) const {
  return getFinalCommonParameters("#time-" + chargedParticleName)(0);
}

void KFCmd::Hypothesis::addMassConstraint(
    const std::string& constraintName, double mass,
    const std::set<std::string>& particleNames) {
  auto constraint = new KFBase::MassConstraint(constraintName, mass);
  addConstraint(constraint);
  for (const auto& name : particleNames) {
    addParticleToConstraint(name, constraintName);
  }
  enableConstraint(constraintName);
}

double KFCmd::Hypothesis::getEnergy() const { return _energy; }

TLorentzVector KFCmd::Hypothesis::getInitialRecoilMomentum(
    const std::set<std::string>& particleNames) const {
  TLorentzVector result(0, 0, 0, getEnergy());
  for (const auto& name : particleNames) {
    result -= getInitialMomentum(name);
  }
  return result;
}

TLorentzVector KFCmd::Hypothesis::getFinalRecoilMomentum(
    const std::set<std::string>& particleNames) const {
  TLorentzVector result(0, 0, 0, getEnergy());
  for (const auto& name : particleNames) {
    result -= getFinalMomentum(name);
  }
  return result;
}

bool KFCmd::Hypothesis::checkMatrixInvertibility(
    const Eigen::MatrixXd& matrix) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
  if (svd.singularValues()(svd.singularValues().size() - 1) == 0) return false;
  return true;
}

Eigen::MatrixXd KFCmd::Hypothesis::inverseMatrix(
    const Eigen::MatrixXd& matrix) {
  return matrix.inverse();
}

bool KFCmd::Hypothesis::fillTrack(const std::string& name, std::size_t index,
                                  const KFCmd::TrPh& data) {
  Eigen::VectorXd par = Eigen::VectorXd::Zero(5);
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(5, 5);
  cov(0, 0) = (data.terr0)[index][0][0];  // p, p
  cov(0, 1) = (data.terr0)[index][0][3];  // p, ctg
  cov(0, 2) = (data.terr0)[index][0][1];  // p, phi
  cov(0, 3) = (data.terr0)[index][0][2];  // p, rho
  cov(0, 4) = (data.terr0)[index][0][4];  // p, z

  cov(1, 0) = (data.terr0)[index][3][0];  // ctg, p
  cov(1, 1) = (data.terr0)[index][3][3];  // ctg, ctg
  cov(1, 2) = (data.terr0)[index][3][1];  // ctg, phi
  cov(1, 3) = (data.terr0)[index][3][2];  // ctg, rho
  cov(1, 4) = (data.terr0)[index][3][4];  // ctg, z

  cov(2, 0) = (data.terr0)[index][1][0];  // phi, p
  cov(2, 1) = (data.terr0)[index][1][3];  // phi, ctg
  cov(2, 2) = (data.terr0)[index][1][1];  // phi, phi
  cov(2, 3) = (data.terr0)[index][1][2];  // phi, rho
  cov(2, 4) = (data.terr0)[index][1][4];  // phi, z

  cov(3, 0) = (data.terr0)[index][2][0];  // rho, p
  cov(3, 1) = (data.terr0)[index][2][3];  // rho, ctg
  cov(3, 2) = (data.terr0)[index][2][1];  // rho, phi
  cov(3, 3) = (data.terr0)[index][2][2];  // rho, rho
  cov(3, 4) = (data.terr0)[index][2][4];  // rho, z

  cov(4, 0) = (data.terr0)[index][4][0];  // z, p
  cov(4, 1) = (data.terr0)[index][4][3];  // z, ctg
  cov(4, 2) = (data.terr0)[index][4][1];  // z, phi
  cov(4, 3) = (data.terr0)[index][4][2];  // z, rho
  cov(4, 4) = (data.terr0)[index][4][4];  // z, z

  if (0 == cov.determinant()) return false;

  par(0) = (data.tptot)[index] * std::sin((data.tth)[index]);
  par(1) = 1. / std::tan((data.tth)[index]);
  par(2) = (data.tphi)[index];
  par(3) = (data.trho)[index];
  par(4) = (data.tz)[index];

  this->setInitialParticleParams(name, par);
  Eigen::MatrixXd inv = cov.inverse();
  this->setParticleInverseErrorMatrix(name, inv);
  return true;
}

bool KFCmd::Hypothesis::fillPhoton(const std::string& name, std::size_t index,
                                   const KFCmd::TrPh& data) {
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
  if (0 == cov.determinant()) return false;

  par(0) = (data.phen)[index];
  par(1) = (data.phrho)[index];
  par(2) = (data.phphi0)[index];
  par(3) = z;

  this->setInitialParticleParams(name, par);
  Eigen::MatrixXd inv = cov.inverse();
  this->setParticleInverseErrorMatrix(name, inv);
  return true;
}

void KFCmd::Hypothesis::setBeamXY(double xbeam, double ybeam) {
  for (auto& el : _constraints) {
      auto cnt = dynamic_cast<KFBase::VertexConstraint*>(el.second);
      if (cnt) {
	switch (cnt->getComponent()) {
	case KFBase::VERTEX_X:
	  cnt->setConstraintValue(-xbeam);
	  break;
	case KFBase::VERTEX_Y:
	  cnt->setConstraintValue(-ybeam);
	  break;
	case KFBase::VERTEX_Z:
	  break;
	}
      }
  }
}
