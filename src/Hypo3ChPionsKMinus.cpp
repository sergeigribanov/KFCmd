#include "Hypo3ChPionsKMinus.hpp"

#include <TDatabasePDG.h>

#include "KMinusMeson.hpp"
#include "PiMinusMeson.hpp"
#include "PiPlusMeson.hpp"

KFCmd::Hypo3ChPionsKMinus::Hypo3ChPionsKMinus(double energy, double magnetField,
                                              long nIter, double tolerance)
    : KFCmd::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addVertex("vtx1");
  addChargedParticle(new KFCmd::PiPlusMeson("pi+_0"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-_0"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi+_1"));
  addChargedParticle(new KFCmd::KMinusMeson("k-"));
  addVertexConstraintsXYZ("pi+_1", "vtx0");
  addVertexConstraintsXYZ("k-", "vtx0");
  addVertexConstraintsXYZ("pi+_0", "vtx1");
  addVertexConstraintsXYZ("pi-_0", "vtx1");
}

KFCmd::Hypo3ChPionsKMinus::~Hypo3ChPionsKMinus() {}
