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
 * @file HypoKsKlPi0.cpp
 *
 * @brief Implementation of HypoKsKlPi0 methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/HypoKsKlPi0.hpp"

using namespace kfcmd::hypos;

HypoKsKlPi0::HypoKsKlPi0(double energy, double magnetField,
                                       long nIter, double tolerance)
  : kfcmd::core::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertexXYZ("vtx0");
  addVertexXYZ("vtx1");
  auto pipl1 = new kfcmd::core::PiPlusMeson("pi+_1");
  addChargedParticle(pipl1);
  auto pimi1 = new kfcmd::core::PiMinusMeson("pi-_1");
  addChargedParticle(pimi1);
  auto g0 = new kfcmd::core::Photon("g0");
  addParticle(g0);
  auto g1 = new kfcmd::core::Photon("g1");
  addParticle(g1);
  addIntermediateNeutralParticle("ks", TDatabasePDG::Instance()->GetParticle(310)->Mass(), "vtx0");
  addParticlePxPyPz("kl", TDatabasePDG::Instance()->GetParticle(130)->Mass());
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addMomentumConstraints("em-vtx0", {getParticle("origin")},
                         {g0, g1, getParticle("kl"), getParticle("ks")});
  addMomentumConstraints("em-vtx1", {getParticle("ks")},
                         {pipl1, pimi1});
  addEnergyConstraint("energy-constraint", {getParticle("origin")},
                      {pipl1, pimi1, g0, g1, getParticle("kl")});
  addOutputVertexConstraintsXYZ("pi+_1", "vtx1");
  addOutputVertexConstraintsXYZ("pi-_1", "vtx1");
  addInputVertexConstraintsXYZ("ks", "vtx1");
  addOutputVertexConstraintsXYZ("g0", "vtx0");
  addOutputVertexConstraintsXYZ("g1", "vtx0");
}

HypoKsKlPi0::~HypoKsKlPi0() {}
