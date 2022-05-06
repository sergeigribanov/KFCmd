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
 * @file Hypo2ChPions2Photons_PiMinusDecay.cpp
 *
 * @brief Implementation of Hypo2ChPions2Photons_PiMinusDecay methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/hypos/Hypo2ChPions2Photons_PiMinusDecay.hpp"

#include "kfcmd/core/PiMinusMeson.hpp"
#include "kfcmd/core/PiPlusMeson.hpp"
#include "kfcmd/core/Muon.hpp"

using namespace kfcmd::hypos;

Hypo2ChPions2Photons_PiMinusDecay::Hypo2ChPions2Photons_PiMinusDecay(double energy,
                                                                     double magneticField,
                                                                     long nIter,
                                                                     double tolerance)
    : kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  addVertex("vtx0");
  addVertex("vtx1");
  auto pip = new kfcmd::core::PiPlusMeson("pi+");
  addChargedParticle(pip);
  auto pim = new kfcmd::core::PiMinusMeson("pi-");
  addChargedParticle(pim);
  auto mum = new kfcmd::core::Muon("mu-");
  addChargedParticle(mum);
  addParticleMassLessThetaPhiE("nu");
  auto ph0 = new kfcmd::core::Photon("g0");
  addPhoton(ph0, "vtx0");
  auto ph1 = new kfcmd::core::Photon("g1");
  addPhoton(ph1, "vtx0");
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-vtx0", {getParticle("origin")}, {pip, pim, ph0, ph1});
  addEnergyMomentumConstraints("em-vtx1", {pip}, {mum, getParticle("nu")});

  addOutputVertexConstraintsXYZ("pi+", "vtx0");
  addOutputVertexConstraintsXYZ("pi-", "vtx0");
  addOutputVertexConstraintsXYZ("mu-", "vtx1");
  addInputVertexConstraintsXYZ("pi-", "vtx1");
}

Hypo2ChPions2Photons_PiMinusDecay::~Hypo2ChPions2Photons_PiMinusDecay() {}