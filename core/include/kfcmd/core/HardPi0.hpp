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
 * @file HardPi0.hpp
 *
 * @brief HardPi0 class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef _KFCMD_PARTICLE_ETHETAPHI_HPP_
#define _KFCMD_PARTICLE_ETHETAPHI_HPP_

#include <kfbase/core/Particle.hpp>

namespace kfcmd {
  namespace core {
    /**
     * Implementation of facility that describes a photon properties in
     * the case of the CMD-3 detector ussage.
     */
    class HardPi0 : public kfbase::core::Particle {
    public:
      //! A constructor
      /*!
       * @param name (photon name)
       */
      explicit HardPi0(const std::string&);
      //! A destructor
      virtual ~HardPi0() = default;
      virtual double calcOutputMomentumComponent(const Eigen::VectorXd&,
                                                 kfbase::core::MOMENT_COMPONENT) const override final;
      virtual double calcInputMomentumComponent(const Eigen::VectorXd&,
                                                 kfbase::core::MOMENT_COMPONENT) const override final;
      virtual Eigen::VectorXd calcOutputDMomentumComponent(const Eigen::VectorXd&,
                                                           kfbase::core::MOMENT_COMPONENT) const override final;
      virtual Eigen::VectorXd calcInputDMomentumComponent(const Eigen::VectorXd&,
                                                           kfbase::core::MOMENT_COMPONENT) const override final;
      virtual Eigen::MatrixXd calcOutputD2MomentumComponent(const Eigen::VectorXd&,
                                                            kfbase::core::MOMENT_COMPONENT) const override final;
      virtual Eigen::MatrixXd calcInputD2MomentumComponent(const Eigen::VectorXd&,
                                                           kfbase::core::MOMENT_COMPONENT) const override final;
      protected:
      static const double pi0Mass_;
    };
  } // namespace core
}  // namespace kfcmd

#endif
