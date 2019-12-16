#ifndef __KFCMD_HYPOTHESIS_HPP__
#define __KFCMD_HYPOTHESIS_HPP__
#include <KFBase/Hypothesis.hpp>
#include <KFBase/VertexConstraint.hpp>
#include <set>
#include <string>

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
  TLorentzVector getInitialRecoilMomentum(const std::set<std::string>&) const;
  TLorentzVector getFinalRecoilMomentum(const std::set<std::string>&) const;
  double getEnergy() const;
  void disableVertex(const std::string&);
  void enableVertex(const std::string&);
  void disableVertexComponent(const std::string&, KFBase::VERTEX_COMPONENT);
  void enableVertexComponent(const std::string&, KFBase::VERTEX_COMPONENT);
  void fixVertexComponent(const std::string&, KFBase::VERTEX_COMPONENT);
  void releaseVertexComponent(const std::string&, KFBase::VERTEX_COMPONENT);
  void disableChargedParticle(const std::string&);
  void enableChargedParticle(const std::string&);
  void disablePhoton(const std::string&);
  void enablePhoton(const std::string&);
  void disableVertexConstraintXYZ(const std::string&);
  void disableVertexConstraintX(const std::string&);
  void disableVertexConstraintY(const std::string&);
  void disableVertexConstraintZ(const std::string&);
  void enableVertexConstraintXYZ(const std::string&);
  void enableVertexConstraintX(const std::string&);
  void enableVertexConstraintY(const std::string&);
  void enableVertexConstraintZ(const std::string&);

 protected:
  void addVertex(const std::string&);
  void addChargedParticle(KFCmd::ChargedParticle*);
  void addPhoton(KFCmd::Photon*, const std::string&);
  void addVertexConstraintsXYZ(const std::string&, const std::string&);
  void addMassConstraint(const std::string&, double,
                         const std::set<std::string>&);

 private:
  double _energy;
  std::set<std::string> _vertices;
};
}  // namespace KFCmd

#endif
