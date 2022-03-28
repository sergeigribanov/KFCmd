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
 * @file HypoKsPiPlusKMinusLost.cpp
 *
 * @brief Implementation of HypoKsPiPlusKMinusLost methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/HypoKsPiPlusKMinusLost.hpp"

#include <TDatabasePDG.h>

#include "kfcmd/KMinusMeson.hpp"
#include "kfcmd/PiMinusMeson.hpp"
#include "kfcmd/PiPlusMeson.hpp"

kfcmd::HypoKsPiPlusKMinusLost::HypoKsPiPlusKMinusLost(double energy, double magnetField,
                                              long nIter, double tolerance)
    : kfcmd::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addVertex("vtx1");
  addChargedParticle(new kfcmd::PiPlusMeson("pi+_0"));
  addChargedParticle(new kfcmd::PiMinusMeson("pi-_0"));
  addChargedParticle(new kfcmd::PiMinusMeson("pi+_1"));
  addParticlePxPyPzE("k-", TDatabasePDG::Instance()->GetParticle(-321)->Mass() * 1000);
  addVertexConstraintsXYZ("pi+_1", "vtx0");
  addVertexConstraintsXYZ("pi+_0", "vtx1");
  addVertexConstraintsXYZ("pi-_0", "vtx1");
  addFlowConstraintsXYZ("ks-flow", "vtx0", "vtx1");
  addParticleToFlow("ks-flow", "pi+_0");
  addParticleToFlow("ks-flow", "pi-_0");
}

kfcmd::HypoKsPiPlusKMinusLost::~HypoKsPiPlusKMinusLost() {}
