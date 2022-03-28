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
 * @file HypoKsKlPi0Pi0Loosen.hpp
 *
 * @brief HypoKsKlPi0Pi0Loosen class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef __KFCMD_HYPOKSKLPI0PI0LOOSEN_HPP__
#define __KFCMD_HYPOKSKLPI0PI0LOOSEN_HPP__
#include "kfcmd/core/Hypothesis.hpp"

namespace kfcmd {
  namespace hypos {
    class HypoKsKlPi0Pi0Loosen : public kfcmd::core::Hypothesis {
    public:
      HypoKsKlPi0Pi0Loosen(double, double, long = 20, double = 1.e-3);
      virtual ~HypoKsKlPi0Pi0Loosen();
    };
  } // namespace hypos
} // namespace kfcmd

#endif
