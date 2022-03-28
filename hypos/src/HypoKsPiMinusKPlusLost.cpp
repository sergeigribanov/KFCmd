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
 * @file HypoKsPiMinusKPlusLost.cpp
 *
 * @brief Implementation of HypoKsPiMinusKPlusLost methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/HypoKsPiMinusKPlusLost.hpp"

#include <TDatabasePDG.h>

#include "kfcmd/core/KPlusMeson.hpp"
#include "kfcmd/core/PiMinusMeson.hpp"
#include "kfcmd/core/PiPlusMeson.hpp"

kfcmd::hypos::HypoKsPiMinusKPlusLost::HypoKsPiMinusKPlusLost(double energy, double magnetField,
                                            long nIter, double tolerance)
    : kfcmd::core::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addVertex("vtx1");
  addChargedParticle(new kfcmd::core::PiPlusMeson("pi+_0"));
  addChargedParticle(new kfcmd::core::PiMinusMeson("pi-_0"));
  addChargedParticle(new kfcmd::core::PiMinusMeson("pi-_1"));
  addParticlePxPyPzE("k+", TDatabasePDG::Instance()->GetParticle(321)->Mass() * 1000);
  addVertexConstraintsXYZ("pi-_1", "vtx0");
  addVertexConstraintsXYZ("pi+_0", "vtx1");
  addVertexConstraintsXYZ("pi-_0", "vtx1");
  addFlowConstraintsXYZ("ks-flow", "vtx0", "vtx1");
  addParticleToFlow("ks-flow", "pi+_0");
  addParticleToFlow("ks-flow", "pi-_0");
}

kfcmd::hypos::HypoKsPiMinusKPlusLost::~HypoKsPiMinusKPlusLost() {}
