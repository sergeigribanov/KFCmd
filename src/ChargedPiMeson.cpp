#include <TDatabasePDG.h>
#include "ChargedPiMeson.hpp"

KFCmd::ChargedPiMeson::ChargedPiMeson(const std::string& name):
  ChargedParticle(name, TDatabasePDG::Instance()->GetParticle(211)->Mass() * 1000) {
}

KFCmd::ChargedPiMeson::~ChargedPiMeson() {
}
