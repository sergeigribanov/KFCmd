#include "PiPlusMeson.hpp"
#include "PiMinusMeson.hpp"
#include "Hypo2ChPions2Photons.hpp"

KFCmd::Hypo2ChPions2Photons::Hypo2ChPions2Photons(double energy, double magnetField,
						  long nIter, double tolerance) :
  KFCmd::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addChargedParticle(new KFCmd::PiPlusMeson("pi+"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-"));
  addPhoton(new KFCmd::Photon("g0"), "vtx0");
  addPhoton(new KFCmd::Photon("g1"), "vtx0");
  addVertexConstraintsXYZ("pi+", "vtx0");
  addVertexConstraintsXYZ("pi-", "vtx0");
}

KFCmd::Hypo2ChPions2Photons::~Hypo2ChPions2Photons() {
}
