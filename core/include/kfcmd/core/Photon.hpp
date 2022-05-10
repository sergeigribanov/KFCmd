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
 * @file Photon.hpp
 *
 * @brief Photon class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef __KFCMDPHOTON_HPP__
#define __KFCMDPHOTON_HPP__

#include <kfbase/core/VertexParticle.hpp>

namespace kfcmd {
  namespace core {
    /**
     * Implementation of facility that describes a photon properties in
     * the case of the CMD-3 detector ussage.
     */
    class Photon : public kfbase::core::VertexParticle {
    public:
      //! A constructor
      /*!
       * @param name (photon name)
       */
      explicit Photon(const std::string&);
      //! A destructor
      virtual ~Photon();
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
      virtual double calcOutputVertexComponent(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const override final;
      virtual double calcInputVertexComponent(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const override final;
      virtual Eigen::VectorXd calcOutputDVertexComponent(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const override final;
      virtual Eigen::VectorXd calcInputDVertexComponent(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const override final;
      virtual Eigen::MatrixXd calcOutputD2VertexComponent(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const override final;
      virtual Eigen::MatrixXd calcInputD2VertexComponent(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const override final;
      double calcDirection(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const;
      Eigen::VectorXd calcDDirection(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const;
      Eigen::MatrixXd calcD2Direction(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const;
      virtual double calcConversionPoint(const Eigen::VectorXd&,
                                         kfbase::core::VERTEX_COMPONENT) const;
      virtual Eigen::VectorXd calcDConversionPoint(const Eigen::VectorXd&,
                                                   kfbase::core::VERTEX_COMPONENT) const;
      virtual Eigen::MatrixXd calcD2ConversionPoint(const Eigen::VectorXd&,
                                                    kfbase::core::VERTEX_COMPONENT) const;
      const TVector3& getInitialDirection() const;
      const TVector3& getFinalDirection() const;
      const TVector3& getInitialConvPoint() const;
      const TVector3& getFinalConvPoint() const;
      virtual void onFitBegin(const Eigen::VectorXd&) override final;
      virtual void onFitEnd(const Eigen::VectorXd&) override final;

    private:
      TVector3 initialDirection_; // !!! remove after debuging
      TVector3 finalDirection_; // !!! remove after debuging
      TVector3 initialConvPoint_;
      TVector3 finalConvPoint_;
    };
  } // namespace core
}  // namespace kfcmd

#endif
