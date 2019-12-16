#include "Electron.hpp"

#include <TDatabasePDG.h>

KFCmd::Electron::Electron(const std::string& name)
    : ChargedParticle(
          name, TDatabasePDG::Instance()->GetParticle(11)->Mass() * 1000, -1) {}

KFCmd::Electron::~Electron() {}
