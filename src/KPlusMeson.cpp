#include <TDatabasePDG.h>
#include "KPlusMeson.hpp"

KFCmd::KPlusMeson::KPlusMeson(const std::string& name) :
  ChargedParticle(name, TDatabasePDG::Instance()->GetParticle(321)->Mass() * 1000, 1) {
}

KFCmd::KPlusMeson::~KPlusMeson() {
}
