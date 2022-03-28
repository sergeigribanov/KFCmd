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
 * @file ParticlePxPyPzE.cpp
 *
 * @brief Implementation of ParticlePxPyPzE methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include <cmath>
#include "kfcmd/ParticlePxPyPzE.hpp"

namespace core = kfbase::core;

kfcmd::ParticlePxPyPzE::ParticlePxPyPzE(const std::string& name, double mass) :
  core::ParticlePxPyPzE(name, mass) {
  setLowerLimit(0, -1100);
  setUpperLimit(0, 1100);
  setLowerLimit(1, -1100);
  setUpperLimit(1, 1100);
  setLowerLimit(2, -1100);
  setUpperLimit(2, 1100);
}

kfcmd::ParticlePxPyPzE::~ParticlePxPyPzE() {}
