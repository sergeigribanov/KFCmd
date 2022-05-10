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
 * @file Hypo2Ks2ChPions_NoKsMasses.cpp
 *
 * @brief Implementation of Hypo2Ks2ChPions_NoKsMasses methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/Hypo2Ks2ChPions_NoKsMasses.hpp"

kfcmd::hypos::Hypo2Ks2ChPions_NoKsMasses::Hypo2Ks2ChPions_NoKsMasses(double energy, double magneticField,
                                                 long nIter, double tolerance)
    : kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertexXYZ("vtx0");
  addVertexXYZ("vtx1");
  addVertexXYZ("vtx2");

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
  addMomentumConstraints("momentum-vtx0", {getParticle("origin")},
                         {pipl0, pimi0, getParticle("ks1"), getParticle("ks2")});
  addMomentumConstraints("momentum-vtx1", {getParticle("ks1")},
                         {pipl1, pimi1});
  addMomentumConstraints("momentum-vtx2", {getParticle("ks2")},
                         {pipl2, pimi2});
  addEnergyConstraint("en-constraint", {getParticle("origin")},
                      {pipl0, pimi0, pipl1, pimi1, pipl2, pimi2});
  addOutputVertexConstraintsXYZ("pi-_0", "vtx0");
  addOutputVertexConstraintsXYZ("pi+_0", "vtx0");
  addOutputVertexConstraintsXYZ("pi+_1", "vtx1");
  addOutputVertexConstraintsXYZ("pi-_1", "vtx1");
  addOutputVertexConstraintsXYZ("pi+_2", "vtx2");
  addOutputVertexConstraintsXYZ("pi-_2", "vtx2");
  addInputVertexConstraintsXYZ("ks1", "vtx1");
  addInputVertexConstraintsXYZ("ks2", "vtx2");
}

kfcmd::hypos::Hypo2Ks2ChPions_NoKsMasses::~Hypo2Ks2ChPions_NoKsMasses() {}
