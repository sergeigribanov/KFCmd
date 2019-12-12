#include <KFBase/MomentumConstraint.hpp>
#include <KFBase/VertexConstraint.hpp>
#include <KFBase/MassConstraint.hpp>
#include <ccgo/CommonParams.hpp>
#include "PiPlusMeson.hpp"
#include "PiMinusMeson.hpp"
#include "Photon.hpp"
#include "Hypo2ChPions2Photons.hpp"

KFCmd::Hypo2ChPions2Photons::Hypo2ChPions2Photons(double energy, double magnetField,
						  long nIter, double tolerance) :
  KFBase::Hypothesis(nIter, tolerance) {
  addCommonParams(new ccgo::CommonParams("vtx-x", 1));
  addCommonParams(new ccgo::CommonParams("vtx-y", 1));
  addCommonParams(new ccgo::CommonParams("vtx-z", 1));
  addCommonParams(new ccgo::CommonParams("t-pi+", 1));
  addCommonParams(new ccgo::CommonParams("t-pi-", 1));
  addConstant("m-field", magnetField);

  auto vtxXPiPl = new KFBase::VertexConstraint("vtx-x-pi+", KFBase::VERTEX_X);
  addConstraint(vtxXPiPl);
  vtxXPiPl->setVertexCommonParams("vtx-x");
  auto vtxXPiMi = new KFBase::VertexConstraint("vtx-x-pi-", KFBase::VERTEX_X);
  addConstraint(vtxXPiMi);
  vtxXPiMi->setVertexCommonParams("vtx-x");

  auto vtxYPiPl = new KFBase::VertexConstraint("vtx-y-pi+", KFBase::VERTEX_Y);
  addConstraint(vtxYPiPl);
  vtxYPiPl->setVertexCommonParams("vtx-y");
  auto vtxYPiMi = new KFBase::VertexConstraint("vtx-y-pi-", KFBase::VERTEX_Y);
  addConstraint(vtxYPiMi);
  vtxYPiMi->setVertexCommonParams("vtx-y");
  
  auto vtxZPiPl = new KFBase::VertexConstraint("vtx-z-pi+", KFBase::VERTEX_Z);
  addConstraint(vtxZPiPl);
  vtxZPiPl->setVertexCommonParams("vtx-z");
  auto vtxZPiMi = new KFBase::VertexConstraint("vtx-z-pi-", KFBase::VERTEX_Z);
  addConstraint(vtxZPiMi);
  vtxZPiMi->setVertexCommonParams("vtx-z");
  
  // auto vtxZG0 = new KFBase::VertexConstraint("vtx-z-g0", KFBase::VERTEX_Z);
  // addConstraint(vtxZG0);
  // vtxZG0->setVertexCommonParams("vtx-z");
  // auto vtxZG1 = new KFBase::VertexConstraint("vtx-z-g1", KFBase::VERTEX_Z);
  // addConstraint(vtxZG1);
  // vtxZG1->setVertexCommonParams("vtx-z");

  // addConstraint(new KFBase::MassConstraint("eta-mass", 547.862));
  
  addConstraint(new KFBase::MomentumConstraint("px", KFBase::MOMENT_X, 0));
  addConstraint(new KFBase::MomentumConstraint("py", KFBase::MOMENT_Y, 0));
  addConstraint(new KFBase::MomentumConstraint("pz", KFBase::MOMENT_Z, 0));
  addConstraint(new KFBase::MomentumConstraint("pe", KFBase::MOMENT_E, energy));
  auto pipl = new KFCmd::PiPlusMeson("pi+");
  addParticle(pipl);
  pipl->setMagnetField("m-field");
  pipl->setTimeParameter("t-pi+");
  auto pimi = new KFCmd::PiMinusMeson("pi-");
  addParticle(pimi);
  pimi->setMagnetField("m-field");
  pimi->setTimeParameter("t-pi-");

  auto g0 = new KFCmd::Photon("g0");
  addParticle(g0);
  g0->setVertexX("vtx-x");
  g0->setVertexY("vtx-y");
  g0->setVertexZ("vtx-z");
  auto g1 = new KFCmd::Photon("g1");
  addParticle(g1);
  g1->setVertexX("vtx-x");
  g1->setVertexY("vtx-y");
  g1->setVertexZ("vtx-z");
  // addParticleToConstraint("g0", "eta-mass");
  // addParticleToConstraint("g1", "eta-mass");
  addParticleToConstraint("pi+", "vtx-x-pi+");
  addParticleToConstraint("pi-", "vtx-x-pi-");
  addParticleToConstraint("pi+", "vtx-y-pi+");
  addParticleToConstraint("pi-", "vtx-y-pi-");
  addParticleToConstraint("pi+", "vtx-z-pi+");
  addParticleToConstraint("pi-", "vtx-z-pi-");
  // addParticleToConstraint("g0", "vtx-z-g0");
  // addParticleToConstraint("g1", "vtx-z-g1"); 
  addParticleToConstraint("pi+", "px");
  addParticleToConstraint("pi+", "py");
  addParticleToConstraint("pi+", "pz");
  addParticleToConstraint("pi+", "pe");
  addParticleToConstraint("pi-", "px");
  addParticleToConstraint("pi-", "py");
  addParticleToConstraint("pi-", "pz");
  addParticleToConstraint("pi-", "pe");
  addParticleToConstraint("g0", "px");
  addParticleToConstraint("g0", "py");
  addParticleToConstraint("g0", "pz");
  addParticleToConstraint("g0", "pe");
  addParticleToConstraint("g1", "px");
  addParticleToConstraint("g1", "py");
  addParticleToConstraint("g1", "pz");
  addParticleToConstraint("g1", "pe");
  enableParticle("pi+");
  enableParticle("pi-");
  enableParticle("g0");
  enableParticle("g1");
  // enableConstraint("eta-mass");
  enableConstraint("px");
  enableConstraint("py");
  enableConstraint("pz");
  enableConstraint("pe");
  enableCommonParams("vtx-x");
  enableCommonParams("vtx-y");
  enableCommonParams("vtx-z");
  enableCommonParams("t-pi+");
  enableCommonParams("t-pi-");
  enableConstraint("vtx-x-pi+");
  enableConstraint("vtx-x-pi-");
  enableConstraint("vtx-y-pi+");
  enableConstraint("vtx-y-pi-");
  enableConstraint("vtx-z-pi+");
  enableConstraint("vtx-z-pi-");
  // enableConstraint("vtx-z-g0");
  // enableConstraint("vtx-z-g1");
}

KFCmd::Hypo2ChPions2Photons::~Hypo2ChPions2Photons() {
}

// TO DO : exception in constraint when used common parameter is not enabled
