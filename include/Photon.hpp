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
#include <KFBase/Particle.hpp>

namespace KFCmd {
/**
 * Implementation of facility that describes a photon properties in
 * the case of the CMD-3 detector ussage.
 */
class Photon : public KFBase::Particle {
 public:
  //! A constructor
  /*!
   * @param name (photon name)
   */
  explicit Photon(const std::string&);
  //! A destructor
  virtual ~Photon();
  //! A vertex X coordinate setter
  /*!
   * This method is used to set the common parameter container,
   * that represetns X coordinate of a photon vertex.
   *
   * @param name (name of a common parameter container)
   */
  void setVertexX(const std::string&);
  //! A vertex Y coordinate setter
  /*!
   * This method is used to set the common parameter container,
   * that represetns Y coordinate of a photon vertex.
   *
   * @param name (name of a common parameter container)
   */
  void setVertexY(const std::string&);
  //! A vertex Z coordinate setter
  /*!
   * This method is used to set the common parameter container,
   * that represetns Z coordinate of a photon vertex.
   *
   * @param name (name of a common parameter container)
   */
  void setVertexZ(const std::string&);
  virtual double calcMomentumComponent(
      const Eigen::VectorXd&, KFBase::MOMENT_COMPONENT) const override final;
  virtual Eigen::VectorXd calcDMomentumComponent(
      const Eigen::VectorXd&, KFBase::MOMENT_COMPONENT) const override final;
  virtual Eigen::MatrixXd calcD2MomentumComponent(
      const Eigen::VectorXd&, KFBase::MOMENT_COMPONENT) const override final;

 private:
  //! A poiner to common parameter conatainer for X coordinate of a vertex
  ccgo::CommonParams* _vertexX;
  //! A poiner to common parameter conatainer for Y coordinate of a vertex
  ccgo::CommonParams* _vertexY;
  //! A poiner to common parameter conatainer for Z coordinate of a vertex
  ccgo::CommonParams* _vertexZ;
};
}  // namespace KFCmd

#endif
