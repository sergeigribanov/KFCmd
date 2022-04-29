#include "kfcmd/hypos/Hypo3Photons.hpp"
#include "kfcmd/core/PiMinusMeson.hpp"
#include "kfcmd/core/PiPlusMeson.hpp"

using namespace kfcmd::hypos;

Hypo3Photons::Hypo3Photons(double energy, double magneticField,
                                       long nIter, double tolerance)
  : kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertex("vtx0");
  auto ph0 = new kfcmd::core::Photon("g0");
  addPhoton(ph0, "vtx0");
  auto ph1 = new kfcmd::core::Photon("g1");
  addPhoton(ph1, "vtx0");
  auto ph2 = new kfcmd::core::Photon("g2");
  addPhoton(ph2, "vtx0");
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-constraint", {getParticle("origin")},
                               {ph0, ph1, ph2});
}

Hypo3Photons::~Hypo3Photons() {}
