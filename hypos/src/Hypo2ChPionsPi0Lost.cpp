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
 * @file Hypo2ChPionsPi0Lost.cpp
 *
 * @brief Implementation of Hypo2ChPionsPi0Lost methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/Hypo2ChPionsPi0Lost.hpp"

#include "kfcmd/core/PiMinusMeson.hpp"
#include "kfcmd/core/PiPlusMeson.hpp"

#include <TDatabasePDG.h>

kfcmd::hypos::Hypo2ChPionsPi0Lost::Hypo2ChPionsPi0Lost(double energy,
						double magneticField,
						long nIter, double tolerance)
  : kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertex("vtx0");
  addChargedParticle(new kfcmd::core::PiPlusMeson("pi+"));
  addChargedParticle(new kfcmd::core::PiMinusMeson("pi-"));
  addParticlePxPyPzE("pi0", TDatabasePDG::Instance()->GetParticle(111)->Mass() * 1000);
  addVertexConstraintsXYZ("pi+", "vtx0");
  addVertexConstraintsXYZ("pi-", "vtx0");
}

kfcmd::hypos::Hypo2ChPionsPi0Lost::~Hypo2ChPionsPi0Lost() {}
