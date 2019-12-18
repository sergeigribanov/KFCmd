#include "AntiMuon.hpp"

#include <TDatabasePDG.h>

KFCmd::AntiMuon::AntiMuon(const std::string& name)
    : ChargedParticle(
          name, TDatabasePDG::Instance()->GetParticle(-13)->Mass() * 1000, -1) {
}

KFCmd::AntiMuon::~AntiMuon() {}
