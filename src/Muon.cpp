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
 * @file Muon.cpp
 *
 * @brief Implementation of Muon methods
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#include "Muon.hpp"

#include <TDatabasePDG.h>

KFCmd::Muon::Muon(const std::string& name)
    : ChargedParticle(
          name, TDatabasePDG::Instance()->GetParticle(13)->Mass() * 1000, -1) {}

KFCmd::Muon::~Muon() {}
