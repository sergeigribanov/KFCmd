#include "Hypo3ChPionsKPlus.hpp"

#include <TDatabasePDG.h>

#include "KPlusMeson.hpp"
#include "PiMinusMeson.hpp"
#include "PiPlusMeson.hpp"

KFCmd::Hypo3ChPionsKPlus::Hypo3ChPionsKPlus(double energy, double magnetField,
                                            long nIter, double tolerance)
    : KFCmd::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addVertex("vtx1");
  addChargedParticle(new KFCmd::PiPlusMeson("pi+_0"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-_0"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-_1"));
  addChargedParticle(new KFCmd::KPlusMeson("k+"));
  addVertexConstraintsXYZ("pi-_1", "vtx0");
  addVertexConstraintsXYZ("k+", "vtx0");
  addVertexConstraintsXYZ("pi+_0", "vtx1");
  addVertexConstraintsXYZ("pi-_0", "vtx1");
}

KFCmd::Hypo3ChPionsKPlus::~Hypo3ChPionsKPlus() {}
