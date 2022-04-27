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
 * @file Hypo2ChPions2Photons_EtaMass.cpp
 *
 * @brief Implementation of Hypo2ChPions2Photons_EtaMass methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include <TDatabasePDG.h>
#include "kfcmd/hypos/Hypo2ChPions2Photons_EtaMass.hpp"
#include "kfcmd/core/PiMinusMeson.hpp"
#include "kfcmd/core/PiPlusMeson.hpp"

using namespace kfcmd::hypos;

Hypo2ChPions2Photons_EtaMass::Hypo2ChPions2Photons_EtaMass(double energy,
                                                  double magneticField,
                                                  long nIter, double tolerance)
    : kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertex("vtx0");
  auto pip = new kfcmd::core::PiPlusMeson("pi+");
  addChargedParticle(pip);
  auto pim = new kfcmd::core::PiMinusMeson("pi-");
  addChargedParticle(pim);
  auto ph0 = new kfcmd::core::Photon("g0");
  addPhoton(ph0, "vtx0");
  auto ph1 = new kfcmd::core::Photon("g1");
  addPhoton(ph1, "vtx0");
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-vtx0", {getParticle("origin")}, {pip, pim, ph0, ph1});
  addVertexConstraintsXYZ("pi+", "vtx0");
  addVertexConstraintsXYZ("pi-", "vtx0");
  addMassConstraint("m-eta-constraint",
                    TDatabasePDG::Instance()->GetParticle(221)->Mass(),
                    {"g0", "g1"});
}

Hypo2ChPions2Photons_EtaMass::~Hypo2ChPions2Photons_EtaMass() {}
