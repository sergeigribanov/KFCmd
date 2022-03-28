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
 * @file Hypo2ChPionsPiPlusLostPiMinus.cpp
 *
 * @brief Implementation of Hypo2ChPionsPiPlusLostPiMinus methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include <TDatabasePDG.h>
#include "kfcmd/Hypo2ChPionsPiPlusLostPiMinus.hpp"

#include "kfcmd/PiMinusMeson.hpp"
#include "kfcmd/PiPlusMeson.hpp"

kfcmd::Hypo2ChPionsPiPlusLostPiMinus::Hypo2ChPionsPiPlusLostPiMinus(double energy, double magnetField, long nIter,
                                  double tolerance)
    : kfcmd::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addChargedParticle(new kfcmd::PiPlusMeson("pi+_0"));
  addParticlePxPyPzE("pi+_1", TDatabasePDG::Instance()->GetParticle(211)->Mass() * 1000);
  addChargedParticle(new kfcmd::PiMinusMeson("pi-_0"));
  addChargedParticle(new kfcmd::PiMinusMeson("pi-_1"));
  addVertexConstraintsXYZ("pi+_0", "vtx0");
  addVertexConstraintsXYZ("pi-_0", "vtx0");
  addVertexConstraintsXYZ("pi-_1", "vtx0");
}

kfcmd::Hypo2ChPionsPiPlusLostPiMinus::~Hypo2ChPionsPiPlusLostPiMinus() {}
