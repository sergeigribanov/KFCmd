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
 * @file HypoKsKlPi0.hpp
 *
 * @brief HypoKsKlPi0 class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef _KFCmd_HypoKsKlPi0_HPP_
#define _KFCmd_HypoKsKlPi0_HPP_

#include "kfcmd/core/Hypothesis.hpp"

namespace kfcmd {
  namespace hypos {
    class HypoKsKlPi0 : public kfcmd::core::Hypothesis {
    public:
      HypoKsKlPi0(double, double, long = 20, double = 1.e-4);
      virtual ~HypoKsKlPi0();
    };
  } // namespace hypos
} // namespace kfcmd

#endif
