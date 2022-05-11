#include "kfcmd/hypos/Hypo3Photons.hpp"

#include <iostream>
using namespace kfcmd::hypos;

Hypo3Photons::Hypo3Photons(double energy,
                           double magneticField,
                           long nIter,
                           double tolerance)
  : kfcmd::core::Hypothesis(energy, magneticField, nIter, tolerance) {
  std::cout << "POINT B1" << std::endl;
  addVertexXYZ("vtx0");
  std::cout << getVertex("vtx0")->getName() << std::endl;
  std::cout << "POINT B2" << std::endl;
  addPhoton("g0", "vtx0");
  std::cout << "POINT B3" << std::endl;
  addPhoton("g1", "vtx0");
  std::cout << "POINT B4" << std::endl;
  addPhoton("g2", "vtx0");
  std::cout << "POINT B5" << std::endl;
  addConstantMomentumParticle("origin", energy, Eigen::Vector3d::Zero());
  addEnergyMomentumConstraints("em-constraint", {getParticle("origin")},
                               {getParticle("g0"),
                                getParticle("g1"),
                                getParticle("g2")});
}

Hypo3Photons::~Hypo3Photons() {}
