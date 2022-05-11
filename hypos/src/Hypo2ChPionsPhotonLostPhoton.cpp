#include "kfcmd/hypos/Hypo2ChPionsPhotonLostPhoton.hpp"

using namespace kfcmd::hypos;

Hypo2ChPionsPhotonLostPhoton::Hypo2ChPionsPhotonLostPhoton(double energy,
                                                           double magneticField,
                                                           long nIter,
                                                           double tolerance) :
  kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertexXYZ("vtx0");
  auto pip = new kfcmd::core::PiPlusMeson("pi+");
  addChargedParticle(pip);
  auto pim = new kfcmd::core::PiMinusMeson("pi-");
  addChargedParticle(pim);
  addPhoton("g0", "vtx0");
  addParticleMassLessThetaPhiE("g1");
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-vtx0", {getParticle("origin")},
                               {pip, pim, getParticle("g0"), getParticle("g1")});
  addOutputVertexConstraintsXYZ("pi+", "vtx0");
  addOutputVertexConstraintsXYZ("pi-", "vtx0");
}

Hypo2ChPionsPhotonLostPhoton::~Hypo2ChPionsPhotonLostPhoton() {}
