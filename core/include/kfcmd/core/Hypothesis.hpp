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
 * @file Hypothesis.hpp
 *
 * @brief Hypothesis class definition
 *
 * @ingroup KFCmd
 *
 * @author Sergei Gribanov
 * Contact: ssgribanov@gmail.com
 *
 */

#ifndef __KFCMD_HYPOTHESIS_HPP__
#define __KFCMD_HYPOTHESIS_HPP__

#include <kfbase/core/Hypothesis.hpp>
#include <kfbase/core/VertexConstraint.hpp>
#include <kfbase/core/ConstantMomentumParticle.hpp>
#include <set>
#include <string>

#include <TVector3.h>

#include "kfcmd/core/ChargedParticle.hpp"
#include "kfcmd/core/Photon.hpp"
#include "kfcmd/core/TrPh.hpp"

namespace kfcmd {
  namespace core {
    /**
     * Implementation of a hypotheis facility for the CMD-3 detector
     */
    class Hypothesis : public kfbase::core::Hypothesis {
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
      Hypothesis(double, double, long = 20, double = 1.e-3);
      //! A destructor
      virtual ~Hypothesis();
      TVector3 calcVertexComponent(const std::string&);
      //! A getter for an initial vertex
      /*!
       * @param vertexName (vertex name)
       */
      TVector3 getInitialVertex(const std::string&) const;
      //! A getter for a final vertex
      /*!
       * @param vertexName (vertex name)
       */
      TVector3 getFinalVertex(const std::string&) const;
      //! A getter for an inital vertex X coordinate by name
      /*!
       * @param vertexName (vertex name)
       */
      double getInitialVertexX(const std::string&) const;
      //! A getter for an initial vertex Y coordinate by name
      /*!
       * @param vertexName (vertex name)
       */
      double getInitialVertexY(const std::string&) const;
      //! A getter for an initial vertex Z coordinate by name
      /*!
       * @param vertexName (vertex name)
       */
      double getInitialVertexZ(const std::string&) const;
      //! A getter for a final vertex X coordinate by name
      /*!
       * @param vertexName (vertex name)
       */
      double getFinalVertexX(const std::string&) const;
      //! A getter for a final vertex Y coordinate by name
      /*!
       * @param vertexName (vertex name)
       */
      double getFinalVertexY(const std::string&) const;
      //! A getter for a final vertex Z coordinate by name
      /*!
       * @param vertexName (vertex name)
       */
      double getFinalVertexZ(const std::string&) const;
      //! A getter for an initial value of a track natural parameter [cm]
      /*!
       * @param chargedParticleName (charged particle name)
       */
      double getInitialChargedParticleTime(const std::string&) const;
      //! A getter for a final value of a track natural parameter [cm]
      /*!
       * @param chargedParticleName (charged particle name)
       */
      double getFinalChargedParticleTime(const std::string&) const;
      //! A method that used to calculate an initial recoil momentum for set of
      //! particles
      /*!
       * @param particleNames (set of particle names)
       */
      TLorentzVector getInitialRecoilMomentum(const std::set<std::string>&) const;
      //! A method that used to calculate a final recoil momentum for set of
      //! particles
      /*!
       * @param particleNames (set of particle names)
       */
      TLorentzVector getFinalRecoilMomentum(const std::set<std::string>&) const;
      //! A center-of-mass energy getter
      double getEnergy() const;
      //! A method is used in ROOT scripts to check a matrix invertibility
      /*!
       * This method was added to the source code as a crutch
       * that allows to suppress warnings during inROOT compilation of
       * code contained Eigen3 things related with matrix inversion.
       *
       * @param matrix (matrix)
       */
      static bool checkMatrixInvertibility(const Eigen::MatrixXd&);
      //! A method is used in ROOT scripts to inverse matrix
      /*!
       * This method was added to the source code as a crutch
       * that allows to suppress warnings during inROOT compilation of
       * code contained Eigen3 things related with matrix inversion.
       *
       * @param matrix (matrix)
       */
      static Eigen::MatrixXd inverseMatrix(const Eigen::MatrixXd&);
      //! A method that used to fill track from TrPh
      bool fillTrack(const std::string&, std::size_t, const TrPh&);
      //! A method that used to fill photon from TrPh
      bool fillPhoton(const std::string&, std::size_t, const TrPh&);
      //! A method that used to disable vertex by name
      /*!
       * This method disabless common parameter containers that
       * correspond to x, y and z coordinates of a vertex.
       *
       * @param vertexName (vertex name)
       */
      void disableVertex(const std::string&);
      //! A method that used to enable vertex by name
      /*!
       * This method enables common parameter containers that
       * correspond to x, y and z coordinates of a vertex.
       *
       * @param vertexName (vertex name)
       */
      void enableVertex(const std::string&);
      //! A method that used to disable vertex component by vertex name
      /*!
       * This method disables common parameter container that corresponds
       * to a specific component of a vertex.
       *
       * @param vertexName (vertex name)
       *
       * @param component (vertex component)
       */
      void disableVertexComponent(const std::string&, kfbase::core::VERTEX_COMPONENT);
      //! A method that used to enable vertex component by vertex name
      /*!
       * This method enables common parameter container that corresponds
       * to a specific component of a vertex.
       *
       * @param vertexName (vertex name)
       *
       * @param component (vertex component)
       */
      void enableVertexComponent(const std::string&, kfbase::core::VERTEX_COMPONENT);
      //! A method that used to fix a specific coordinate of a vertex
      /*!
       * @param vertexName (vertex name)
       *
       * @param value (value of coordinate)
       *
       * @param component (vertex component)
       */
      void fixVertexComponent(const std::string&, double, kfbase::core::VERTEX_COMPONENT);
      //! A method that used to release a specific coordinate of a vertex
      /*!
       * @param vertexName (vertex name)
       *
       * @param value (value of coordinate)
       *
       * @param component (vertex component)
       */
      void releaseVertexComponent(const std::string&, kfbase::core::VERTEX_COMPONENT);
      // void fixTrackNaturalParameter(const std::string&, double);
      // void releaseTrackNaturalParameter(const std::string&);

      //! A method that used to disable a charged particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void disableChargedParticle(const std::string&);
      //! A method that used to enable a charged particle by name
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void enableChargedParticle(const std::string&);
      //! A method that used to disable photon by name
      /*!
       * @param photonName (photon name)
       */
      void disablePhoton(const std::string&);
      //! A method that used to enable photon by name
      /*!
       * @param photonName (photon name)
       */
      void enablePhoton(const std::string&);
      //! A method that used to disable X, Y and Z vertex constraint for a certain
      //! charged particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void disableVertexConstraintXYZ(const std::string&);
      void disableFlowConstraintXYZ(const std::string&);
      //! A method that used to disable X vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void disableVertexConstraintX(const std::string&);
      //! A method that used to disable Y vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void disableVertexConstraintY(const std::string&);
      //! A method that used to disable Z vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void disableVertexConstraintZ(const std::string&);
      //! A method that used to enable X, Y and Z vertex constraint for a certain
      //! charged particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void enableVertexConstraintXYZ(const std::string&);
      void enableFlowConstraintXYZ(const std::string&);
      void addEnergyConstraint(const std::string&,
                               const std::set<kfbase::core::Particle*>&,
                               const std::set<kfbase::core::Particle*>&);
      void addMomentumConstraints(const std::string&,
                                  const std::set<kfbase::core::Particle*>&,
                                  const std::set<kfbase::core::Particle*>&);
      void addEnergyMomentumConstraints(const std::string&,
                                        const std::set<kfbase::core::Particle*>&,
                                        const std::set<kfbase::core::Particle*>&);
      void enableEnergyMomentumConstraints(const std::string&);
      void disableEnergyMomentumConstraints(const std::string&);
      //! A method that used to enable X vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void enableVertexConstraintX(const std::string&);
      //! A method that used to enable Y vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void enableVertexConstraintY(const std::string&);
      //! A method that used to enable Z vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void enableVertexConstraintZ(const std::string&);

      void setBeamXY(double, double);

      void setParticleAngularConstraintAxis(const std::string&, const TVector3&);
      void setAngularConstraintSigma(const std::string&,  double);
      void setInitialVertex(const std::string&, const Eigen::Vector3d&);
      void setInitialVertexX(const std::string&, double);
      void setInitialVertexY(const std::string&, double);
      void setInitialVertexZ(const std::string&, double);

    protected:
      //! A method that used to add vertex to a hypothesis
      /*!
       * @param vertexName (vertex name)
       */
      void addVertex(const std::string&);
      void setInitialVertexRandomly(const std::string&, double = 1.e-2);
      //! A method that used to add charged particle to a hypothesis
      /*!
       * @param particle (pointer to ChargedParticle object)
       */
      void addChargedParticle(kfcmd::core::ChargedParticle*);
      //! A method that used to add a photon to a hypothesis
      /*!
       * @param photon (Pointer to a Photon object)
       *
       * @param vertexName (photon vertex)
       */
      void addPhoton(kfcmd::core::Photon*, const std::string&);
      void addConstantMomentumParticle(const std::string&, double,
                                       const Eigen::Vector3d&);
      void addIntermediateNeutralParticle(const std::string&, double, const std::string&);
      void addParticlePxPyPzE(const std::string&, double);
      void addParticleMassLessThetaPhiE(const std::string&);
      //! A method that used to add vertex constraint for a certain charged particle
      //! to a hypothesis
      /*!
       * @param chargedParticleName (charged particle name)
       *
       * @param vertexName (vertex name)
       */
      void addVertexConstraintsXYZ(const std::string&, const std::string&);
      void addDoubleParticleAngularConstraint(const std::string&,
                                              const std::string&,
                                              const std::string&,
                                              double);
      void addParticleAngularConstraint(const std::string&,
                                        const std::string&,
                                        double);
      //! A method that used to add mass constraint to a hypothesis
      /*!
       * @param constraintName (constraint name)
       *
       * @param mass (mass)
       *
       * @param particleNames (set of particle names)
       */
      void addMassConstraint(const std::string&, double,
                             const std::set<std::string>&);

    private:
      //! A center-of-mass energy
      double _energy;
      //! A set of vertex names
      std::set<std::string> _vertices;
    };
  } // namespace core
}  // namespace kfcmd

#endif
