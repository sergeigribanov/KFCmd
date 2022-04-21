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
 * @file ChargedParticle.hpp
 *
 * @brief ChargedParticle class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef __KFCMD_CHARGEDPARTICLE_HPP__
#define __KFCMD_CHARGEDPARTICLE_HPP__
#include <kfbase/core/VertexParticle.hpp>
#include <kfbase/newtonian_opt/CommonParams.hpp>

namespace kfcmd {
  namespace core {
    /**
     * Implementation of a facility that describes charged particle
     * properties in the case of the CMD-3 detector ussage.
     */
    class ChargedParticle : public kfbase::core::VertexParticle {
    public:
      //! A constructor
      /*!
       * @param name (particle name)
       *
       * @param mass (particle mass)
       *
       * @param charge (particle charge)
       */
      ChargedParticle(const std::string&, double, double);
      //! A destructor
      virtual ~ChargedParticle();
      //! A magnet field setter
      /*!
       * This method tells to particle name of the optimizer constant that represent
       * a magnetic field.
       *
       * @param name (name of a constant)
       */
      void setMagneticField(const std::string&);
      void setBeamX(const std::string&);
      void setBeamY(const std::string&);
      virtual double calcMomentumComponent(
                                           const Eigen::VectorXd&, kfbase::core::MOMENT_COMPONENT) const override final;
      virtual Eigen::VectorXd calcDMomentumComponent(
                                                     const Eigen::VectorXd&, kfbase::core::MOMENT_COMPONENT) const override final;
      virtual Eigen::MatrixXd calcD2MomentumComponent(
                                                      const Eigen::VectorXd&, kfbase::core::MOMENT_COMPONENT) const override final;
      virtual double calcVertexComponent(
                                         const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const override final;
      virtual Eigen::VectorXd calcDVertexComponent(
                                                   const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const override final;
      virtual Eigen::MatrixXd calcD2VertexComponent(
                                                    const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const override final;

    protected:
      //! A magnetic field getter
      double getMagneticField() const;
      
    private:
      //! A constant that proportional to the light velocity
      static const double _c;
      //! An iterator that points to a magnet field constant
      std::unordered_map<std::string, double>::const_iterator _magnetFieldIterator;
      std::unordered_map<std::string, double>::const_iterator _beamX;
      std::unordered_map<std::string, double>::const_iterator _beamY;
    };
  }  // namespace core
} // namespace kfcmd

#endif
