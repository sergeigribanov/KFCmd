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
 * @file Positron.hpp
 *
 * @brief Positron class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef __KFCMD_POSITRON_HPP__
#define __KFCMD_POSITRON_HPP__
#include "ChargedParticle.hpp"

namespace KFCmd {
/**
 * Positron implementation
 */
class Positron : public ChargedParticle {
 public:
  //! A constructor
  /*!
   * @param name (positron name)
   */
  explicit Positron(const std::string&);
  //! A destructor
  virtual ~Positron();
};
}  // namespace KFCmd

#endif
