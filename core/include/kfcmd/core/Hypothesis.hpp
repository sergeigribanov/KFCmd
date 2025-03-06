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
#include <kfbase/core/OutputVertexConstraint.hpp>
#include <kfbase/core/InputVertexConstraint.hpp>
#include <kfbase/core/ConstantMomentumParticle.hpp>

#include <set>
#include <string>

#include <TVector3.h>
#include <TDatabasePDG.h>

#include "kfcmd/core/ChargedParticle.hpp"
#include "kfcmd/core/Photon.hpp"
#include "kfcmd/core/TrPh.hpp"
#include "kfcmd/core/KMinusMeson.hpp"
#include "kfcmd/core/KPlusMeson.hpp"
#include "kfcmd/core/PiMinusMeson.hpp"
#include "kfcmd/core/PiPlusMeson.hpp"
#include "kfcmd/core/Muon.hpp"
#include "kfcmd/core/AntiMuon.hpp"
#include "kfcmd/core/Electron.hpp"
#include "kfcmd/core/Positron.hpp"

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
      Hypothesis(double, double, long = 20, double = 1.e-4);
      //! A destructor
      virtual ~Hypothesis();
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
      //! A method that used to fill alternative parametrized photon from TrPh
      bool fillAltPhoton(const std::string&, std::size_t, const TrPh&);
      //! A method that used to fill alternative parametrized photon from TrPh (taking into account LXe strips)
      bool fillAltBSPhoton(const std::string&, std::size_t, const TrPh&);
      //! A method that used to disable X, Y and Z vertex constraint for a certain
      //! charged particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void disableOutputVertexConstraintXYZ(const std::string&);
      void disableInputVertexConstraintXYZ(const std::string&);
      //! A method that used to disable X vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void disableOutputVertexConstraintX(const std::string&);
      void disableInputVertexConstraintX(const std::string&);
      //! A method that used to disable Y vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void disableOutputVertexConstraintY(const std::string&);
      void disableInputVertexConstraintY(const std::string&);
      //! A method that used to disable Z vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void disableOutputVertexConstraintZ(const std::string&);
      void disableInputVertexConstraintZ(const std::string&);
      //! A method that used to enable X, Y and Z vertex constraint for a certain
      //! charged particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void enableOutputVertexConstraintXYZ(const std::string&);
      void enableInputVertexConstraintXYZ(const std::string&);
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
      void enableOutputVertexConstraintX(const std::string&);
      void enableInputVertexConstraintX(const std::string&);
      //! A method that used to enable Y vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void enableOutputVertexConstraintY(const std::string&);
      void enableInputVertexConstraintY(const std::string&);
      //! A method that used to enable Z vertex constraint for a certain charged
      //! particle
      /*!
       * @param chargedParticleName (charged particle name)
       */
      void enableOutputVertexConstraintZ(const std::string&);
      void enableInputVertexConstraintZ(const std::string&);

      void setBeamXY(double, double);

      void setParticleAngularConstraintAxis(const std::string&, const TVector3&);
      void setAngularConstraintSigma(const std::string&,  double);

    protected:
      //! A method that used to add vertex to a hypothesis
      /*!
       * @param vertexName (vertex name)
       */
      void addVertexXYZ(const std::string&);
      //! A method that used to add charged particle to a hypothesis
      /*!
       * @param particle (pointer to ChargedParticle object)
       */
      void addChargedParticle(kfcmd::core::ChargedParticle*);
      void addPhoton(const std::string& , const std::string&);
      void addAltPhoton(const std::string&);
      void addConstantMomentumParticle(const std::string&, double,
                                       const Eigen::Vector3d&);
      void addIntermediateNeutralParticle(const std::string&, double, const std::string&);
      void addParticlePxPyPz(const std::string&, double);
      void addParticleMassLessThetaPhiE(const std::string&);
      //! A method that used to add vertex constraint for a certain charged particle
      //! to a hypothesis
      /*!
       * @param chargedParticleName (charged particle name)
       *
       * @param vertexName (vertex name)
       */
      void addOutputVertexConstraintsXYZ(const std::string&, const std::string&);
      void addInputVertexConstraintsXYZ(const std::string&, const std::string&);
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
    };
  } // namespace core
}  // namespace kfcmd

#endif
