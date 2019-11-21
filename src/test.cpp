#include <iostream>
#include <ccgo/Optimizer.hpp>
#include <KFBase/KFMomentumConstraint.hpp>
#include "KFCmdPhoton.hpp"

int main() {
  KFBase::KFMomentumConstraint cpx("px", KFBase::KFMOMENT_X, 0);
  KFBase::KFMomentumConstraint cpy("py", KFBase::KFMOMENT_Y, 0);
  KFBase::KFMomentumConstraint cpz("pz", KFBase::KFMOMENT_Z, 0);
  KFBase::KFMomentumConstraint cpe("pe", KFBase::KFMOMENT_E, 1000);
  KFCmd::KFCmdPhoton g0("g0");
  KFCmd::KFCmdPhoton g1("g1");
  ccgo::Optimizer opt;
  opt.addTarget(&g0);
  opt.addTarget(&g1);
  opt.addConstraint(&cpx);
  opt.addConstraint(&cpy);
  opt.addConstraint(&cpz);
  opt.addConstraint(&cpe);
  Eigen::VectorXd xg0(5);
  xg0(0) = 501;
  xg0(1) = 0;
  xg0(2) = 1;
  xg0(3) = 0;
  xg0(4) = 2;
  Eigen::VectorXd xg1(5);
  xg1(0) = 499;
  xg1(1) = 0;
  xg1(2) = 3.14 + 1;
  xg1(3) = 0;
  xg1(4) = 3.14 - 2;
  g0.setInitialParameters(xg0);
  g1.setInitialParameters(xg1);
  opt.enableTarget("g0");
  opt.enableTarget("g1");
  opt.enableConstraint("px");
  opt.enableConstraint("py");
  opt.enableConstraint("pz");
  opt.enableConstraint("pe");
  opt.addTargetToConstraint("g0", "px");
  opt.addTargetToConstraint("g0", "py");
  opt.addTargetToConstraint("g0", "pz");
  opt.addTargetToConstraint("g0", "pe");
  opt.addTargetToConstraint("g1", "px");
  opt.addTargetToConstraint("g1", "py");
  opt.addTargetToConstraint("g1", "pz");
  opt.addTargetToConstraint("g1", "pe");
  Eigen::MatrixXd im(5, 5);
  im(0, 0) = 10.1;
  im(1, 1) = 10.1;
  im(2, 2) = 10.1;
  im(3, 3) = 10.1;
  im(4, 4) = 10.1;
  g0.setInverseErrorMatrix(im);
  g1.setInverseErrorMatrix(im);
  opt.optimize();
  std::cout << "---" << std::endl;
  std::cout << "error code = " << opt.getErrorCode() << std::endl;
  std::cout << "chi square = " << opt.getChiSquare() << std::endl;
  std::cout << "---" << std::endl;
  std::cout << g0.getFinalParameters() << std::endl;
  std::cout << "---" << std::endl;
  std::cout << g1.getFinalParameters() << std::endl;
  std::cout << "----" << std::endl;
  std::cout << "g0 momentum:" << std::endl;
  g0.getFinalMomentum().Print();
  std::cout << "g1 momentum:" << std::endl;
  g1.getFinalMomentum().Print();
  return 0;
}
