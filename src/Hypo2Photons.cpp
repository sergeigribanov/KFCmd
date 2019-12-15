#include "PiPlusMeson.hpp"
#include "PiMinusMeson.hpp"
#include "Hypo2Photons.hpp"

KFCmd::Hypo2Photons::Hypo2Photons(double energy, long nIter, double tolerance) :
  KFCmd::Hypothesis(energy, 0., nIter, tolerance) {
  addVertex("vtx0");
  addPhoton(new KFCmd::Photon("g0"), "vtx0");
  addPhoton(new KFCmd::Photon("g1"), "vtx0");
}

KFCmd::Hypo2Photons::~Hypo2Photons() {
}
