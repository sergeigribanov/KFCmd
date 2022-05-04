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
 * @file Hypo2Ks2ChPions.cpp
 *
 * @brief Implementation of Hypo2Ks2ChPions methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include <TDatabasePDG.h>
#include "kfcmd/hypos/Hypo2Ks2ChPions.hpp"
#include "kfcmd/core/KPlusMeson.hpp"
#include "kfcmd/core/PiMinusMeson.hpp"
#include "kfcmd/core/PiPlusMeson.hpp"

kfcmd::hypos::Hypo2Ks2ChPions::Hypo2Ks2ChPions(double energy, double magnetField,
                                                 long nIter, double tolerance)
    : kfcmd::core::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addVertex("vtx1");
  addVertex("vtx2");

  auto pipl0 = new kfcmd::core::PiPlusMeson("pi+_0");
  addChargedParticle(pipl0);
  auto pimi0 = new kfcmd::core::PiMinusMeson("pi-_0");
  addChargedParticle(pimi0);
  auto pipl1 = new kfcmd::core::PiPlusMeson("pi+_1");
  addChargedParticle(pipl1);
  auto pimi1 = new kfcmd::core::PiMinusMeson("pi-_1");
  addChargedParticle(pimi1);
  auto pipl2 = new kfcmd::core::PiPlusMeson("pi+_2");
  addChargedParticle(pipl2);
  auto pimi2 = new kfcmd::core::PiMinusMeson("pi-_2");
  addChargedParticle(pimi2);
  addIntermediateNeutralParticle("ks1", TDatabasePDG::Instance()->GetParticle(310)->Mass(), "vtx0");
  addIntermediateNeutralParticle("ks2", TDatabasePDG::Instance()->GetParticle(310)->Mass(), "vtx0");
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("momentum-vtx0", {getParticle("origin")},
                               {pipl0, pimi0, getParticle("ks1"), getParticle("ks2")});
  addEnergyMomentumConstraints("momentum-vtx1", {getParticle("ks1")},
                               {pipl1, pimi1});
  addEnergyMomentumConstraints("momentum-vtx2", {getParticle("ks2")},
                               {pipl2, pimi2});

  addVertexConstraintsXYZ("pi-_0", "vtx0");
  addVertexConstraintsXYZ("pi+_0", "vtx0");

  addVertexConstraintsXYZ("pi+_1", "vtx1");
  addVertexConstraintsXYZ("pi-_1", "vtx1");

  addVertexConstraintsXYZ("pi+_2", "vtx2");
  addVertexConstraintsXYZ("pi-_2", "vtx2");

  addVertexConstraintsXYZ("ks1", "vtx1");
  addVertexConstraintsXYZ("ks2", "vtx2");
}

kfcmd::hypos::Hypo2Ks2ChPions::~Hypo2Ks2ChPions() {}
