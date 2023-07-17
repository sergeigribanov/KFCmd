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
 * @file Hypo2ChPions4AltPhotons.cpp
 *
 * @brief Implementation of Hypo2ChPions4AltPhotons methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/Hypo2ChPions4AltPhotons.hpp"

using namespace kfcmd::hypos;

Hypo2ChPions4AltPhotons::Hypo2ChPions4AltPhotons(double energy,
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
  addAltPhoton("g2");
  addAltPhoton("g3");
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-vtx0", {getParticle("origin")},
                               {pip, pim,
                                getParticle("g0"),
                                getParticle("g1"),
                                getParticle("g2"),
                                getParticle("g3")});
  addOutputVertexConstraintsXYZ("pi+", "vtx0");
  addOutputVertexConstraintsXYZ("pi-", "vtx0");
  const double pi0_mass = TDatabasePDG::Instance()->GetParticle(111)->Mass();

  addMassConstraint("pi0-mass-g0-g1", pi0_mass, {"g0", "g1"});
  addMassConstraint("pi0-mass-g0-g2", pi0_mass, {"g0", "g2"});
  addMassConstraint("pi0-mass-g0-g3", pi0_mass, {"g0", "g3"});
  addMassConstraint("pi0-mass-g1-g2", pi0_mass, {"g1", "g2"});
  addMassConstraint("pi0-mass-g1-g3", pi0_mass, {"g1", "g3"});
  addMassConstraint("pi0-mass-g2-g3", pi0_mass, {"g2", "g3"});
  disableConstraint("pi0-mass-g0-g1");
  disableConstraint("pi0-mass-g0-g2");
  disableConstraint("pi0-mass-g0-g3");
  disableConstraint("pi0-mass-g1-g2");
  disableConstraint("pi0-mass-g1-g3");
  disableConstraint("pi0-mass-g2-g3");
}

Hypo2ChPions4AltPhotons::~Hypo2ChPions4AltPhotons() {}
