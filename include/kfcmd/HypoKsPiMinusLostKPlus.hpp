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
 * @file HypoKsPiMinusLostKPlus.hpp
 *
 * @brief HypoKsPiMinusLostKPlus class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef __KFCMD_HYPOPIMINUSLOSTKPLUS_HPP__
#define __KFCMD_HYPOPIMINUSLOSTKPLUS_HPP__
#include "kfcmd/Hypothesis.hpp"

namespace kfcmd {
/**
 * Implementation of Ks(Ks==pi+pi-)pi-(lost)K+ hypothesis
 */
class HypoKsPiMinusLostKPlus : public Hypothesis {
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
  HypoKsPiMinusLostKPlus(double, double, long = 20, double = 1.e-3);
  //! A destructor
  virtual ~HypoKsPiMinusLostKPlus();
};
}  // namespace kfcmd

#endif
