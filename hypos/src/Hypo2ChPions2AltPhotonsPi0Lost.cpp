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
 * @file Hypo2ChPions2AltPhotonsPi0Lost.cpp
 *
 * @brief Implementation of Hypo2ChPions2AltPhotonsPi0Lost methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/Hypo2ChPions2AltPhotonsPi0Lost.hpp"

using namespace kfcmd::hypos;

Hypo2ChPions2AltPhotonsPi0Lost::Hypo2ChPions2AltPhotonsPi0Lost(double energy,
                       double magneticField,
                       long nIter,
                       double tolerance)
    : kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertexXYZ("vtx0");
  auto pip = new kfcmd::core::PiPlusMeson("pi+");
  addChargedParticle(pip);
  auto pim = new kfcmd::core::PiMinusMeson("pi-");
  addChargedParticle(pim);
  addAltPhoton("g0");
  addAltPhoton("g1");
  addParticlePxPyPz("pi0", TDatabasePDG::Instance()->GetParticle(111)->Mass());
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-vtx0", {getParticle("origin")},
                               {pip, pim, getParticle("g0"), getParticle("g1"),
                                getParticle("pi0")});
  addOutputVertexConstraintsXYZ("pi+", "vtx0");
  addOutputVertexConstraintsXYZ("pi-", "vtx0");
  // addMassConstraint("pi0-mass", TDatabasePDG::Instance()->GetParticle(111)->Mass(),  {"g0", "g1"});
}

Hypo2ChPions2AltPhotonsPi0Lost::~Hypo2ChPions2AltPhotonsPi0Lost() {}
