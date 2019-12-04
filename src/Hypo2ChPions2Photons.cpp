#include <KFBase/MomentumConstraint.hpp>
#include <KFBase/VertexConstraint.hpp>
#include <ccgo/CommonParams.hpp>
#include "ChargedPiMeson.hpp"
#include "Photon.hpp"
#include "Hypo2ChPions2Photons.hpp"

KFCmd::Hypo2ChPions2Photons::Hypo2ChPions2Photons(double energy) {
  addCommonParams(new ccgo::CommonParams("vtx-z", 1));
  auto vtxZPiPl = new KFBase::VertexConstraint("vtx-z-pi+", KFBase::VERTEX_Z);
  addConstraint(vtxZPiPl);
  vtxZPiPl->setVertexCommonParams("vtx-z");
  auto vtxZPiMi = new KFBase::VertexConstraint("vtx-z-pi-", KFBase::VERTEX_Z);
  addConstraint(vtxZPiMi);
  vtxZPiMi->setVertexCommonParams("vtx-z");
  auto vtxZG0 = new KFBase::VertexConstraint("vtx-z-g0", KFBase::VERTEX_Z);
  addConstraint(vtxZG0);
  vtxZG0->setVertexCommonParams("vtx-z");
  auto vtxZG1 = new KFBase::VertexConstraint("vtx-z-g1", KFBase::VERTEX_Z);
  addConstraint(vtxZG1);
  vtxZG1->setVertexCommonParams("vtx-z");
  addConstraint(new KFBase::MomentumConstraint("px", KFBase::MOMENT_X, 0));
  addConstraint(new KFBase::MomentumConstraint("py", KFBase::MOMENT_Y, 0));
  addConstraint(new KFBase::MomentumConstraint("pz", KFBase::MOMENT_Z, 0));
  addConstraint(new KFBase::MomentumConstraint("pe", KFBase::MOMENT_E, energy));
  addParticle(new KFCmd::ChargedPiMeson("pi+"));
  addParticle(new KFCmd::ChargedPiMeson("pi-"));
  addParticle(new KFCmd::Photon("g0"));
  addParticle(new KFCmd::Photon("g1"));
  addParticleToConstraint("pi+", "vtx-z-pi+");
  addParticleToConstraint("pi-", "vtx-z-pi-");
  addParticleToConstraint("g0", "vtx-z-g0");
  addParticleToConstraint("g1", "vtx-z-g1");
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
  enableConstraint("px");
  enableConstraint("py");
  enableConstraint("pz");
  enableConstraint("pe");
  enableCommonParams("vtx-z");
  enableConstraint("vtx-z-pi+");
  enableConstraint("vtx-z-pi-");
  enableConstraint("vtx-z-g0");
  enableConstraint("vtx-z-g1");
}

KFCmd::Hypo2ChPions2Photons::~Hypo2ChPions2Photons() {
}
