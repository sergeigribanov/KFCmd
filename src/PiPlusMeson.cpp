#include "PiPlusMeson.hpp"

#include <TDatabasePDG.h>

KFCmd::PiPlusMeson::PiPlusMeson(const std::string& name)
    : ChargedParticle(
          name, TDatabasePDG::Instance()->GetParticle(211)->Mass() * 1000, 1) {}

KFCmd::PiPlusMeson::~PiPlusMeson() {}
