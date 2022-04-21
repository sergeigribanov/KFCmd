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
 * @file PiMinusMeson.cpp
 *
 * @brief Implementation of PiMinusMeson methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "kfcmd/core/PiMinusMeson.hpp"

#include <TDatabasePDG.h>

kfcmd::core::PiMinusMeson::PiMinusMeson(const std::string& name)
    : ChargedParticle(
          name, TDatabasePDG::Instance()->GetParticle(-211)->Mass(), -1) {}

kfcmd::core::PiMinusMeson::~PiMinusMeson() {}
