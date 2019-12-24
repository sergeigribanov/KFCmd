/*
 * KFCmd library
 * See COPYRIGHT file at the top of the source tree.
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
 * @file Hypo4ChPions.cpp
 *
 * @brief Implementation of Hypo4ChPions methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "Hypo4ChPions.hpp"

#include "PiMinusMeson.hpp"
#include "PiPlusMeson.hpp"

KFCmd::Hypo4ChPions::Hypo4ChPions(double energy, double magnetField, long nIter,
                                  double tolerance)
    : KFCmd::Hypothesis(energy, magnetField, nIter, tolerance) {
  addVertex("vtx0");
  addChargedParticle(new KFCmd::PiPlusMeson("pi+_0"));
  addChargedParticle(new KFCmd::PiPlusMeson("pi+_1"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-_0"));
  addChargedParticle(new KFCmd::PiMinusMeson("pi-_1"));
  addVertexConstraintsXYZ("pi+_0", "vtx0");
  addVertexConstraintsXYZ("pi+_1", "vtx0");
  addVertexConstraintsXYZ("pi-_0", "vtx0");
  addVertexConstraintsXYZ("pi-_1", "vtx0");
}

KFCmd::Hypo4ChPions::~Hypo4ChPions() {}
