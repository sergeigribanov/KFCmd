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
 * @file Hypo3ChPionsKPlus.hpp
 *
 * @brief Hypo3ChPionsKPlus class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef __KFCMD_HYPO3CHPIONSKPLUS_HPP__
#define __KFCMD_HYPO3CHPIONSKPLUS_HPP__
#include "Hypothesis.hpp"

namespace KFCmd {
/**
 * Implementation of K+pi-pi+pi- hypothesis
 */
class Hypo3ChPionsKPlus : public Hypothesis {
 public:
  //! A constructor
  /*!
   * @param energy (center-of-mass energy)
   *
   * @param magneticField (magnetic field)
   *
   * @param nIter (maximum number of iterations)
   *
   * @param tolerance (optimization tolerance)
   */
  Hypo3ChPionsKPlus(double, double, long = 20, double = 1.e-3);
  //! A destructor
  virtual ~Hypo3ChPionsKPlus();
};
}  // namespace KFCmd

#endif
