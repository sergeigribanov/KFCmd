/*
 * KFCmd library
 * See LICENSE file at the top of the source tree.
 *
 * This product includes software developed by the
 * CMD-3 collaboration (https://cmd.inp.nsk.su/).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

/**
 * @file HypoKsKlPi0.cpp
 *
 * @brief Implementation of HypoKsKlPi0 methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/HypoKsKlPi0.hpp"
#include "kfcmd/core/PiMinusMeson.hpp"
#include "kfcmd/core/PiPlusMeson.hpp"

#include <TDatabasePDG.h>

kfcmd::hypos::HypoKsKlPi0::HypoKsKlPi0(double energy, double magnetField,
                                       long nIter, double tolerance)
  : kfcmd::core::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addVertex("vtx1");
  addChargedParticle(new kfcmd::core::PiPlusMeson("pi+_1"));
  addChargedParticle(new kfcmd::core::PiMinusMeson("pi-_1"));
  addPhoton(new kfcmd::core::Photon("g0"), "vtx0");
  addPhoton(new kfcmd::core::Photon("g1"), "vtx0");
  addParticlePxPyPzE("kl", TDatabasePDG::Instance()->GetParticle(130)->Mass() * 1000);
  addVertexConstraintsXYZ("pi+_1", "vtx1");
  addVertexConstraintsXYZ("pi-_1", "vtx1");
  addFlowConstraintsXYZ("ks-flow", "vtx0", "vtx1");
  addParticleToFlow("ks-flow", "pi+_1");
  addParticleToFlow("ks-flow", "pi-_1");
  addMassConstraint("m-pi0-constraint",
                    TDatabasePDG::Instance()->GetParticle(111)->Mass() * 1000,
                    {"g0", "g1"});
  disableConstraint("m-pi0-constraint");
}

kfcmd::hypos::HypoKsKlPi0::~HypoKsKlPi0() {}
