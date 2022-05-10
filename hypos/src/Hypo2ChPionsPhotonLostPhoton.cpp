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
  auto ph0 = new kfcmd::core::Photon("g0");
  addParticle(ph0);
  addParticleMassLessThetaPhiE("g1");
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-vtx0", {getParticle("origin")},
                               {pip, pim, ph0, getParticle("g1")});
  addOutputVertexConstraintsXYZ("pi+", "vtx0");
  addOutputVertexConstraintsXYZ("pi-", "vtx0");
  addOutputVertexConstraintsXYZ("g0", "vtx0");
}

Hypo2ChPionsPhotonLostPhoton::~Hypo2ChPionsPhotonLostPhoton() {}
