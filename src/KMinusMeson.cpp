#include <TDatabasePDG.h>
#include "KMinusMeson.hpp"

KFCmd::KMinusMeson::KMinusMeson(const std::string& name) :
  ChargedParticle(name, TDatabasePDG::Instance()->GetParticle(-321)->Mass() * 1000, -1) {
}

KFCmd::KMinusMeson::~KMinusMeson() {
}

