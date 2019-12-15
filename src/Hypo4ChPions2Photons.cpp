#include "PiPlusMeson.hpp"
#include "PiMinusMeson.hpp"
#include "Hypo4ChPions2Photons.hpp"

KFCmd::Hypo4ChPions2Photons::Hypo4ChPions2Photons(double energy, double magnetField,
						  long nIter, double tolerance) :
  KFCmd::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addChargedParticle(new KFCmd::PiPlusMeson("pi+_0"));
  addChargedParticle(new KFCmd::PiPlusMeson("pi+_1"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-_0"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-_1"));
  addPhoton(new KFCmd::Photon("g0"), "vtx0");
  addPhoton(new KFCmd::Photon("g1"), "vtx0");
  addVertexConstraintsXYZ("pi+_0", "vtx0");
  addVertexConstraintsXYZ("pi+_1", "vtx0");
  addVertexConstraintsXYZ("pi-_0", "vtx0");
  addVertexConstraintsXYZ("pi-_1", "vtx0");
}

KFCmd::Hypo4ChPions2Photons::~Hypo4ChPions2Photons() {
}
