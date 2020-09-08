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

#include "Hypo2ChPionsPi0Lost.hpp"

#include "PiMinusMeson.hpp"
#include "PiPlusMeson.hpp"

#include <TDatabasePDG.h>

KFCmd::Hypo2ChPionsPi0Lost::Hypo2ChPionsPi0Lost(double energy,
						double magneticField,
						long nIter, double tolerance)
  : KFCmd::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertex("vtx0");
  addChargedParticle(new KFCmd::PiPlusMeson("pi+"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-"));
  addParticlePxPyPzE("pi0", TDatabasePDG::Instance()->GetParticle(111)->Mass() * 1000);
  addVertexConstraintsXYZ("pi+", "vtx0");
  addVertexConstraintsXYZ("pi-", "vtx0");
}

KFCmd::Hypo2ChPionsPi0Lost::~Hypo2ChPionsPi0Lost() {}
