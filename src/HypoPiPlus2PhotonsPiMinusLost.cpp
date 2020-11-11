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
 * @file HypoPiPlus2PhotonsPiMinusLost.cpp
 *
 * @brief Implementation of HypoPiPlus2PhotonsPiMinusLost methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "HypoPiPlus2PhotonsPiMinusLost.hpp"

#include <TDatabasePDG.h>

#include "PiMinusMeson.hpp"
#include "PiPlusMeson.hpp"

KFCmd::HypoPiPlus2PhotonsPiMinusLost::HypoPiPlus2PhotonsPiMinusLost(double energy,
                                                  double magneticField,
                                                  long nIter, double tolerance)
    : KFCmd::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertex("vtx0");
  addChargedParticle(new KFCmd::PiPlusMeson("pi+"));
  addParticlePxPyPzE(new KFCmd::ParticlePxPyPzE("pi-", TDatabasePDG::Instance()->GetParticle(-211)->Mass() * 1000));
  addPhoton(new KFCmd::Photon("g0"), "vtx0");
  addPhoton(new KFCmd::Photon("g1"), "vtx0");
  addVertexConstraintsXYZ("pi+", "vtx0");
}

KFCmd::HypoPiPlus2PhotonsPiMinusLost::~HypoPiPlus2PhotonsPiMinusLost() {}
