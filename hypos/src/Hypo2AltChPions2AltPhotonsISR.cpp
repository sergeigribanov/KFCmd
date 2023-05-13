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
 * @file Hypo2AltChPions2AltPhotons.cpp
 *
 * @brief Implementation of Hypo2AltChPions2AltPhotons methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/Hypo2AltChPions2AltPhotons.hpp"
#include <TDatabasePDG.h>

using namespace kfcmd::hypos;

Hypo2AltChPions2AltPhotons::Hypo2AltChPions2AltPhotons(double energy,
                                                       double magneticField,
                                                       long nIter,
                                                       double tolerance)
    : kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  addParticlePxPyPz("pi-", TDatabasePDG::Instance()->GetParticle(-211)->Mass());
  addParticlePxPyPz("pi+", TDatabasePDG::Instance()->GetParticle(211)->Mass());
  addAltPhoton("g0");
  addAltPhoton("g1");
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-vtx0",
                               {getParticle("origin")},
                               {getParticle("pi-"),
                                getParticle("pi+"),
                                getParticle("g0"),
                                getParticle("g1")});
}

Hypo2AltChPions2AltPhotons::~Hypo2AltChPions2AltPhotons() {}
