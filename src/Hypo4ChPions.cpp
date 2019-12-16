#include "PiPlusMeson.hpp"
#include "PiMinusMeson.hpp"
#include "Hypo4ChPions.hpp"

KFCmd::Hypo4ChPions::Hypo4ChPions(double energy, double magnetField,
						  long nIter, double tolerance) :
  KFCmd::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addChargedParticle(new KFCmd::PiPlusMeson("pi+_0"));
  addChargedParticle(new KFCmd::PiPlusMeson("pi+_1"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-_0"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-_1"));
  addVertexConstraintsXYZ("pi+_0", "vtx0");
  addVertexConstraintsXYZ("pi+_1", "vtx0");
  addVertexConstraintsXYZ("pi-_0", "vtx0");
  addVertexConstraintsXYZ("pi-_1", "vtx0");
}

KFCmd::Hypo4ChPions::~Hypo4ChPions() {
}
