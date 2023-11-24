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
 * @file Hypo6Photons.cpp
 *
 * @brief Implementation of Hypo6Photons methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/Hypo6Photons.hpp"

using namespace kfcmd::hypos;

Hypo6Photons::Hypo6Photons(double energy,
		 double magneticField,
		 long nIter,
		 double tolerance)
    : kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertexXYZ("vtx0");
  addPhoton("g0", "vtx0");
  addPhoton("g1", "vtx0");
  addPhoton("g2", "vtx0");
  addPhoton("g3", "vtx0");
  addPhoton("g4", "vtx0");
  addPhoton("g5", "vtx0");
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-vtx0", {getParticle("origin")},
                               {getParticle("g0"), getParticle("g1"),
				getParticle("g2"), getParticle("g3"),
				getParticle("g4"), getParticle("g5")});
  
  const double pi0_mass = TDatabasePDG::Instance()->GetParticle(111)->Mass();
  addMassConstraint("pi0-mass-g2-g3", pi0_mass, {"g2", "g3"});
  addMassConstraint("pi0-mass-g4-g5", pi0_mass, {"g4", "g5"});
  const double eta_mass = TDatabasePDG::Instance()->GetParticle(221)->Mass();
  addMassConstraint("eta-mass-g0-g1", eta_mass, {"g0", "g1"});
  disableConstraint("eta-mass-g0-g1");
}

Hypo6Photons::~Hypo6Photons() {}
