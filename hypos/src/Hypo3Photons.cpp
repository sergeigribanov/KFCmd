#include "kfcmd/hypos/Hypo3Photons.hpp"

using namespace kfcmd::hypos;

Hypo3Photons::Hypo3Photons(double energy,
                           double magneticField,
                           long nIter,
                           double tolerance)
  : kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertexXYZ("vtx0");
  auto ph0 = new kfcmd::core::Photon("g0");
  addParticle(ph0);
  auto ph1 = new kfcmd::core::Photon("g1");
  addParticle(ph1);
  auto ph2 = new kfcmd::core::Photon("g2");
  addParticle(ph2);
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-constraint", {getParticle("origin")},
                               {ph0, ph1, ph2});
  addOutputVertexConstraintsXYZ("g0", "vtx0");
  addOutputVertexConstraintsXYZ("g1", "vtx0");
  addOutputVertexConstraintsXYZ("g2", "vtx0");
}

Hypo3Photons::~Hypo3Photons() {}
