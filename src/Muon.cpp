#include "Muon.hpp"

#include <TDatabasePDG.h>

KFCmd::Muon::Muon(const std::string& name)
    : ChargedParticle(
          name, TDatabasePDG::Instance()->GetParticle(13)->Mass() * 1000, -1) {}

KFCmd::Muon::~Muon() {}
