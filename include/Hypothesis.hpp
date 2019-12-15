#ifndef __KFCMD_HYPOTHESIS_HPP__
#define __KFCMD_HYPOTHESIS_HPP__
#include <string>
#include <set>
#include <KFBase/Hypothesis.hpp>
#include <KFBase/VertexConstraint.hpp>
#include "ChargedParticle.hpp"
#include "Photon.hpp"

namespace KFCmd {
  class Hypothesis : public KFBase::Hypothesis {
  public:
    Hypothesis(double, double, long = 20, double = 1.e-3);
    virtual ~Hypothesis();
    double getInitialVertexX(const std::string&) const;
    double getInitialVertexY(const std::string&) const;
    double getInitialVertexZ(const std::string&) const;
    double getFinalVertexX(const std::string&) const;
    double getFinalVertexY(const std::string&) const;
    double getFinalVertexZ(const std::string&) const;
    double getInitialChargedParticleTime(const std::string&) const;
    double getFinalChargedParticleTime(const std::string&) const;
  protected:
    void addVertex(const std::string&);
    void disableVertex(const std::string&);
    void enableVertex(const std::string&);
    void disableVertexComponent(const std::string&, KFBase::VERTEX_COMPONENT);
    void enableVertexComponent(const std::string&, KFBase::VERTEX_COMPONENT);
    void fixVertexComponent(const std::string&, KFBase::VERTEX_COMPONENT);
    void releaseVertexComponent(const std::string&, KFBase::VERTEX_COMPONENT);
    void addChargedParticle(KFCmd::ChargedParticle*);
    void disableChargedParticle(const std::string&);
    void enableChargedParticle(const std::string&);
    void addPhoton(KFCmd::Photon*, const std::string&);
    void disablePhoton(const std::string&);
    void enablePhoton(const std::string&);
    void addVertexConstraintsXYZ(const std::string&, const std::string&);
    void disableVertexConstraintXYZ(const std::string&);
    void disableVertexConstraintX(const std::string&);
    void disableVertexConstraintY(const std::string&);
    void disableVertexConstraintZ(const std::string&);
    void enableVertexConstraintXYZ(const std::string&);
    void enableVertexConstraintX(const std::string&);
    void enableVertexConstraintY(const std::string&);
    void enableVertexConstraintZ(const std::string&);
  private:
    std::set<std::string> _vertices;
    std::set<std::string> _chargedParticles;
    std::set<std::string> _photons;
  };
}

#endif
