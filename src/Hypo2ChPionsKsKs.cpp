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
 * @file Hypo2ChPionsKsKs.cpp
 *
 * @brief Implementation of Hypo2ChPionsKsKs methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/Hypo2ChPionsKsKs.hpp"

#include <TDatabasePDG.h>

#include "kfcmd/KPlusMeson.hpp"
#include "kfcmd/PiMinusMeson.hpp"
#include "kfcmd/PiPlusMeson.hpp"

kfcmd::Hypo2ChPionsKsKs::Hypo2ChPionsKsKs(double energy, double magnetField,
                                            long nIter, double tolerance)
    : kfcmd::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addVertex("vtx1");
  addVertex("vtx2");
  addChargedParticle(new kfcmd::PiPlusMeson("pi+_0"));
  addChargedParticle(new kfcmd::PiMinusMeson("pi-_0"));
  addChargedParticle(new kfcmd::PiPlusMeson("pi+_1"));
  addChargedParticle(new kfcmd::PiMinusMeson("pi-_1"));
  addChargedParticle(new kfcmd::PiPlusMeson("pi+_2"));
  addChargedParticle(new kfcmd::PiMinusMeson("pi-_2"));
  addVertexConstraintsXYZ("pi-_0", "vtx0");
  addVertexConstraintsXYZ("pi+_0", "vtx0");
  addVertexConstraintsXYZ("pi+_1", "vtx1");
  addVertexConstraintsXYZ("pi-_1", "vtx1");
  addVertexConstraintsXYZ("pi+_2", "vtx2");
  addVertexConstraintsXYZ("pi-_2", "vtx2");
  addFlowConstraintsXYZ("ks1-flow", "vtx1", "vtx0");
  addParticleToFlow("ks1-flow", "pi+_1");
  addParticleToFlow("ks1-flow", "pi-_1");
  addFlowConstraintsXYZ("ks2-flow", "vtx2", "vtx0");
  addParticleToFlow("ks2-flow", "pi+_2");
  addParticleToFlow("ks2-flow", "pi-_2");
}

kfcmd::Hypo2ChPionsKsKs::~Hypo2ChPionsKsKs() {}
