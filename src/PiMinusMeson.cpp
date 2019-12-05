#include <TDatabasePDG.h>
#include "PiMinusMeson.hpp"

KFCmd::PiMinusMeson::PiMinusMeson(const std::string& name) :
  ChargedParticle(name, TDatabasePDG::Instance()->GetParticle(-211)->Mass() * 1000, -1) {
}

KFCmd::PiMinusMeson::~PiMinusMeson() {
}
