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
 * @file HypoPiMinus2PhotonsPiPlusLost.cpp
 *
 * @brief Implementation of HypoPiMinus2PhotonsPiPlusLost methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "HypoPiMinus2PhotonsPiPlusLost.hpp"

#include <TDatabasePDG.h>

#include "PiMinusMeson.hpp"
#include "PiPlusMeson.hpp"

KFCmd::HypoPiMinus2PhotonsPiPlusLost::HypoPiMinus2PhotonsPiPlusLost(double energy,
                                                  double magneticField,
                                                  long nIter, double tolerance)
    : KFCmd::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertex("vtx0");
  addParticlePxPyPzE("pi+", TDatabasePDG::Instance()->GetParticle(211)->Mass() * 1000);
  addChargedParticle(new KFCmd::PiMinusMeson("pi-"));
  addPhoton(new KFCmd::Photon("g0"), "vtx0");
  addPhoton(new KFCmd::Photon("g1"), "vtx0");
  addVertexConstraintsXYZ("pi-", "vtx0");
}

KFCmd::HypoPiMinus2PhotonsPiPlusLost::~HypoPiMinus2PhotonsPiPlusLost() {}
