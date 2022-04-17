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
 * @file Hypo3ChPionsKPlus.cpp
 *
 * @brief Implementation of Hypo3ChPionsKPlus methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/Hypo3ChPionsKPlus.hpp"

#include <TDatabasePDG.h>

#include "kfcmd/core/KPlusMeson.hpp"
#include "kfcmd/core/PiMinusMeson.hpp"
#include "kfcmd/core/PiPlusMeson.hpp"

kfcmd::hypos::Hypo3ChPionsKPlus::Hypo3ChPionsKPlus(double energy, double magnetField,
                                            long nIter, double tolerance)
    : kfcmd::core::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addVertex("vtx1");
  auto pipl0 = new kfcmd::core::PiPlusMeson("pi+_0");
  addChargedParticle(pipl0);
  auto pimi0 = new kfcmd::core::PiMinusMeson("pi-_0");
  addChargedParticle(pimi0);
  auto pimi1 = new kfcmd::core::PiMinusMeson("pi-_1");
  addChargedParticle(pimi1);
  auto kpl = new kfcmd::core::KPlusMeson("k+");
  addChargedParticle(kpl);
  addIntermediateNeutralParticle("ks", TDatabasePDG::Instance()->GetParticle(310)->Mass() * 1000, "vtx0");
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-vtx0", {getParticle("origin")},
                               {pimi1, kpl, getParticle("ks")});
  addEnergyMomentumConstraints("em-vtx1", {getParticle("ks")},
                               {pipl0, pimi0});

  // addMomentumConstraints("momentum-vtx0", {getParticle("origin")},
  //                        {pimi1, kpl, getParticle("ks")});
  // addMomentumConstraints("momentum-vtx1", {getParticle("ks")},
  //                        {pipl0, pimi0});
  // addEnergyConstraint("energy-constraint", {getParticle("origin")},
  //                     {pimi1, kpl, pipl0, pimi0});

  addVertexConstraintsXYZ("pi-_1", "vtx0");
  addVertexConstraintsXYZ("k+", "vtx0");
  addVertexConstraintsXYZ("pi+_0", "vtx1");
  addVertexConstraintsXYZ("pi-_0", "vtx1");
  addVertexConstraintsXYZ("ks", "vtx1");
}

kfcmd::hypos::Hypo3ChPionsKPlus::~Hypo3ChPionsKPlus() {}
