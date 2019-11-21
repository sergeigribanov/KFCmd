#include <iostream>
#include "Hypo2Photons.hpp"

int main() {
  KFCmd::Hypo2Photons hypo(1000);
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
  hypo.setInitialParticleParams("g0", xg0);
  hypo.setInitialParticleParams("g1", xg1);
  Eigen::MatrixXd im(5, 5);
  im(0, 0) = 10.1;
  im(1, 1) = 10.1;
  im(2, 2) = 10.1;
  im(3, 3) = 10.1;
  im(4, 4) = 10.1;
  hypo.setParticleInverseErrorMatrix("g0", im);
  hypo.setParticleInverseErrorMatrix("g1", im);
  hypo.optimize();
  std::cout << "---" << std::endl;
  std::cout << "error code = " << hypo.getErrorCode() << std::endl;
  std::cout << "chi square = " << hypo.getChiSquare() << std::endl;
  std::cout << "---" << std::endl;
  std::cout << hypo.getFinalParameters("g0") << std::endl;
  std::cout << "---" << std::endl;
  std::cout << hypo.getFinalParameters("g1") << std::endl;
  std::cout << "----" << std::endl;
  std::cout << "g0 momentum:" << std::endl;
  hypo.getFinalMomentum("g0").Print();
  std::cout << "g1 momentum:" << std::endl;
  hypo.getFinalMomentum("g1").Print();
  return 0;
}
