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
 * @file Hypo3Photons.cpp
 *
 * @brief Implementation of Hypo3Photons methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/Hypo3Photons.hpp"

#include "kfcmd/PiMinusMeson.hpp"
#include "kfcmd/PiPlusMeson.hpp"

kfcmd::Hypo3Photons::Hypo3Photons(double energy, long nIter, double tolerance)
    : kfcmd::Hypothesis(energy, 0., nIter, tolerance) {
  addVertex("vtx0");
  addPhoton(new kfcmd::Photon("g0"), "vtx0");
  addPhoton(new kfcmd::Photon("g1"), "vtx0");
  addPhoton(new kfcmd::Photon("g2"), "vtx0");
}

kfcmd::Hypo3Photons::~Hypo3Photons() {}
