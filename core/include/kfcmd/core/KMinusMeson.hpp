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
 * @file KMinusMeson.hpp
 *
 * @brief KMinusMeson class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef __KFCMD_KMINUSMESON_HPP__
#define __KFCMD_KMINUSMESON_HPP__
#include "kfcmd/core/ChargedParticle.hpp"

namespace kfcmd {
  namespace core {
    /**
     * K- meson implementation
     */
    class KMinusMeson : public ChargedParticle {
    public:
      //! A constructor
      /*!
       * @param name (K- name)
       */
      explicit KMinusMeson(const std::string&);
      //! A destructor
      virtual ~KMinusMeson();
    };
  } // namespace core
}  // namespace kfcmd

#endif
