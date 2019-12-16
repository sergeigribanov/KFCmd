#include "Hypothesis.hpp"

#include <KFBase/MassConstraint.hpp>
#include <KFBase/MomentumConstraint.hpp>
#include <ccgo/CommonParams.hpp>

KFCmd::Hypothesis::Hypothesis(double energy, double magnetField, long nIter,
                              double tolerance)
    : KFBase::Hypothesis(nIter, tolerance), _energy(energy) {
  addConstant("#m-field", magnetField);
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

void KFCmd::Hypothesis::addVertex(const std::string& vertexName) {
  if (_vertices.find(vertexName) != _vertices.end()) {
    // TO DO : exception;
    return;
  }
  const std::string vx = "#" + vertexName + "-x";
  const std::string vy = "#" + vertexName + "-y";
  const std::string vz = "#" + vertexName + "-z";
  addCommonParams(new ccgo::CommonParams(vx, 1));
  addCommonParams(new ccgo::CommonParams(vy, 1));
  addCommonParams(new ccgo::CommonParams(vz, 1));
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

void KFCmd::Hypothesis::fixVertexComponent(const std::string& vertexName,
                                           KFBase::VERTEX_COMPONENT component) {
  switch (component) {
    case KFBase::VERTEX_X:
      _commonParams.at("#" + vertexName + "-x")->fixParameter(0);
      break;
    case KFBase::VERTEX_Y:
      _commonParams.at("#" + vertexName + "-y")->fixParameter(0);
      break;
    case KFBase::VERTEX_Z:
      _commonParams.at("#" + vertexName + "-z")->fixParameter(0);
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

void KFCmd::Hypothesis::addChargedParticle(KFCmd::ChargedParticle* particle) {
  addParticle(particle);
  particle->setMagnetField("#m-field");
  const std::string timeParameter = "#time-" + particle->getName();
  addCommonParams(new ccgo::CommonParams(timeParameter, 1));
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
  photon->setVertexX("#" + vertexName + "-x");
  photon->setVertexY("#" + vertexName + "-y");
  photon->setVertexZ("#" + vertexName + "-z");
  enableParticle(photon->getName());
  addParticleToConstraint(photon->getName(), "#constraint-px");
  addParticleToConstraint(photon->getName(), "#constraint-py");
  addParticleToConstraint(photon->getName(), "#constraint-pz");
  addParticleToConstraint(photon->getName(), "#constraint-pe");
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
  vtxX->setVertexCommonParams("#" + vertexName + "-x");
  auto vtxY = new KFBase::VertexConstraint(
      "#" + chargedParticleName + "-constraint-y", KFBase::VERTEX_Y);
  addConstraint(vtxY);
  vtxY->setVertexCommonParams("#" + vertexName + "-y");
  auto vtxZ = new KFBase::VertexConstraint(
      chargedParticleName + "-constraint-z", KFBase::VERTEX_Z);
  addConstraint(vtxZ);
  vtxZ->setVertexCommonParams("#" + vertexName + "-z");
  addParticleToConstraint(chargedParticleName, vtxX->getName());
  addParticleToConstraint(chargedParticleName, vtxY->getName());
  addParticleToConstraint(chargedParticleName, vtxZ->getName());
  enableConstraint(vtxX->getName());
  enableConstraint(vtxY->getName());
  enableConstraint(vtxZ->getName());
}

void KFCmd::Hypothesis::disableVertexConstraintXYZ(
    const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-x");
  disableConstraint("#" + chargedParticleName + "-constraint-y");
  disableConstraint("#" + chargedParticleName + "-constraint-z");
}

void KFCmd::Hypothesis::disableVertexConstraintX(
    const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-x");
}

void KFCmd::Hypothesis::disableVertexConstraintY(
    const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-y");
}

void KFCmd::Hypothesis::disableVertexConstraintZ(
    const std::string& chargedParticleName) {
  disableConstraint("#" + chargedParticleName + "-constraint-z");
}

void KFCmd::Hypothesis::enableVertexConstraintXYZ(
    const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-x");
  enableConstraint("#" + chargedParticleName + "-constraint-y");
  enableConstraint("#" + chargedParticleName + "-constraint-z");
}

void KFCmd::Hypothesis::enableVertexConstraintX(
    const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-x");
}

void KFCmd::Hypothesis::enableVertexConstraintY(
    const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-y");
}

void KFCmd::Hypothesis::enableVertexConstraintZ(
    const std::string& chargedParticleName) {
  enableConstraint("#" + chargedParticleName + "-constraint-z");
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
    const std::string& constraintName, double value,
    const std::set<std::string>& particleNames) {
  auto constraint = new KFBase::MassConstraint(constraintName, value);
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
