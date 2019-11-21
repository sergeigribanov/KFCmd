#include <KFBase/MomentumConstraint.hpp>
#include "ChargedPiMeson.hpp"
#include "Hypo2ChPions.hpp"

KFCmd::Hypo2ChPions::Hypo2ChPions(double energy) {
  addConstraint(new KFBase::MomentumConstraint("px", KFBase::MOMENT_X, 0));
  addConstraint(new KFBase::MomentumConstraint("py", KFBase::MOMENT_Y, 0));
  addConstraint(new KFBase::MomentumConstraint("pz", KFBase::MOMENT_Z, 0));
  addConstraint(new KFBase::MomentumConstraint("pe", KFBase::MOMENT_E, energy));
  addParticle(new KFCmd::ChargedPiMeson("pi+"));
  addParticle(new KFCmd::ChargedPiMeson("pi-"));
  addParticleToConstraint("pi+", "px");
  addParticleToConstraint("pi+", "py");
  addParticleToConstraint("pi+", "pz");
  addParticleToConstraint("pi+", "pe");
  addParticleToConstraint("pi-", "px");
  addParticleToConstraint("pi-", "py");
  addParticleToConstraint("pi-", "pz");
  addParticleToConstraint("pi-", "pe");
  enableParticle("pi+");
  enableParticle("pi-");
  enableConstraint("px");
  enableConstraint("py");
  enableConstraint("pz");
  enableConstraint("pe");
}

KFCmd::Hypo2ChPions::~Hypo2ChPions() { 
}
