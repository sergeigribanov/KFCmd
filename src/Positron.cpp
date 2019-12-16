#include <TDatabasePDG.h>
#include "Positron.hpp"

KFCmd::Positron::Positron(const std::string& name) :
  ChargedParticle(name, TDatabasePDG::Instance()->GetParticle(-11)->Mass() * 1000, 1) {
}

KFCmd::Positron::~Positron() {
}
