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
 * @file Hypo2ChPions2Photons_PiPlusDecay.hpp
 *
 * @brief Hypo2ChPions2Photons_PiPlusDecay class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef _KFCmd_Hypo2ChPions2Photons_PiPlusDecay_HPP_
#define _KFCmd_Hypo2ChPions2Photons_PiPlusDecay_HPP_
#include "kfcmd/core/Hypothesis.hpp"

namespace kfcmd {
  namespace hypos {
    /**
     * Implementation of (pi+, pi-, gamma, gamma, pi+ --> mu+nu_mu) hypothesis
     */
    class Hypo2ChPions2Photons_PiPlusDecay : public kfcmd::core::Hypothesis {
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
      Hypo2ChPions2Photons_PiPlusDecay(double, double, long = 20, double = 1.e-4);
      //! A destructor
      virtual ~Hypo2ChPions2Photons_PiPlusDecay();
    };
  } // namespace core
}  // namespace kfcmd

#endif
