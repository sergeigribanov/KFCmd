#include "kfcmd/core/BGOLogNormalPhoton.hpp"
#include <cmath>
#include <iostream>

namespace kfcmd {
  namespace core {

    BGOLogNormalPhoton::BGOLogNormalPhoton(const std::string& name)
      : kfbase::core::Particle(name, 4, 0., 0.),
	vertex_(nullptr),
	E_meas_(0.),
	endcap_(0)
    {
      setLowerLimit(0, 1.e-2);
      setUpperLimit(0, 1.1);
      setLowerLimit(1, 10);
      setUpperLimit(1, 100);
      setPeriod(2, 0, 2 * M_PI);
      setLowerLimit(2, -1000 * M_PI);
      setUpperLimit(2, 1000 * M_PI);
      setLowerLimit(3, -50);
      setUpperLimit(3, 50);
    }

    BGOLogNormalPhoton::~BGOLogNormalPhoton() {}

    // ----------------------------------------------------------------------
    // Conversion point (identical to Photon)
    // ----------------------------------------------------------------------
    double BGOLogNormalPhoton::calcConversionPoint(const Eigen::VectorXd& x,
						   kfbase::core::VERTEX_COMPONENT component) const {
      const long bi = getBeginIndex();
      if (bi < 0 || bi + 3 >= x.size()) return 0.;
      switch (component) {
      case kfbase::core::VERTEX_X: return x(bi + 1) * std::cos(x(bi + 2));
      case kfbase::core::VERTEX_Y: return x(bi + 1) * std::sin(x(bi + 2));
      case kfbase::core::VERTEX_Z: return x(bi + 3);
      }
      return 0.;
    }

    Eigen::VectorXd BGOLogNormalPhoton::calcDConversionPoint(const Eigen::VectorXd& x,
							     kfbase::core::VERTEX_COMPONENT component) const {
      const long bi = getBeginIndex();
      Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
      if (bi < 0 || bi + 3 >= x.size()) return result;
      switch (component) {
      case kfbase::core::VERTEX_X:
	result(bi + 1) = std::cos(x(bi + 2));
	result(bi + 2) = -x(bi + 1) * std::sin(x(bi + 2));
	break;
      case kfbase::core::VERTEX_Y:
	result(bi + 1) = std::sin(x(bi + 2));
	result(bi + 2) = x(bi + 1) * std::cos(x(bi + 2));
	break;
      case kfbase::core::VERTEX_Z:
	result(bi + 3) = 1.;
	break;
      }
      return result;
    }

    Eigen::MatrixXd BGOLogNormalPhoton::calcD2ConversionPoint(const Eigen::VectorXd& x,
							      kfbase::core::VERTEX_COMPONENT component) const {
      const long bi = getBeginIndex();
      Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
      if (bi < 0 || bi + 3 >= x.size()) return result;
      const long rhoInd = bi + 1;
      const long phiInd = bi + 2;
      switch (component) {
      case kfbase::core::VERTEX_X:
	result(phiInd, phiInd) = -x(rhoInd) * std::cos(x(phiInd));
	result(rhoInd, phiInd) = -std::sin(x(phiInd));
	result(phiInd, rhoInd) = result(rhoInd, phiInd);
	break;
      case kfbase::core::VERTEX_Y:
	result(phiInd, phiInd) = -x(rhoInd) * std::sin(x(phiInd));
	result(rhoInd, phiInd) = std::cos(x(phiInd));
	result(phiInd, rhoInd) = result(rhoInd, phiInd);
	break;
      case kfbase::core::VERTEX_Z:
	break;
      }
      return result;
    }

    // ----------------------------------------------------------------------
    // Direction (identical to Photon)
    // ----------------------------------------------------------------------
    double BGOLogNormalPhoton::calcDirection(const Eigen::VectorXd& x,
					     kfbase::core::VERTEX_COMPONENT component) const {
      if (!vertex_) return 0.;
      const long bi = getBeginIndex();
      if (bi < 0 || bi + 3 >= x.size()) return 0.;
      Eigen::Vector3d vi;
      vi << calcConversionPoint(x, kfbase::core::VERTEX_X) -
	vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_X),
	calcConversionPoint(x, kfbase::core::VERTEX_Y) -
	vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Y),
	calcConversionPoint(x, kfbase::core::VERTEX_Z) -
	vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Z);
      double norm = vi.norm();
      if (norm < 1e-12) return 0.;
      vi /= norm;
      switch (component) {
      case kfbase::core::VERTEX_X: return vi(0);
      case kfbase::core::VERTEX_Y: return vi(1);
      case kfbase::core::VERTEX_Z: return vi(2);
      }
      return 0.;
    }

    Eigen::VectorXd BGOLogNormalPhoton::calcDDirection(const Eigen::VectorXd& x,
						       kfbase::core::VERTEX_COMPONENT component) const {
      if (!vertex_) return Eigen::VectorXd::Zero(x.size());
      const long bi = getBeginIndex();
      if (bi < 0 || bi + 3 >= x.size()) return Eigen::VectorXd::Zero(x.size());
      Eigen::Vector3d vi;
      vi << calcConversionPoint(x, kfbase::core::VERTEX_X) -
	vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_X),
	calcConversionPoint(x, kfbase::core::VERTEX_Y) -
	vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Y),
	calcConversionPoint(x, kfbase::core::VERTEX_Z) -
	vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Z);
      double norm = vi.norm();
      if (norm < 1e-12) return Eigen::VectorXd::Zero(x.size());
      vi /= norm;
      Eigen::MatrixXd dvi(x.size(), 3);
      dvi << calcDConversionPoint(x, kfbase::core::VERTEX_X) -
	vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_X),
        calcDConversionPoint(x, kfbase::core::VERTEX_Y) -
	vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_Y),
        calcDConversionPoint(x, kfbase::core::VERTEX_Z) -
	vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_Z);
      dvi /= norm;
      const Eigen::VectorXd tv = dvi * vi;
      const int idx = int(component);
      return dvi.col(idx) - vi(idx) * tv;
    }

    Eigen::MatrixXd BGOLogNormalPhoton::calcD2Direction(const Eigen::VectorXd& x,
							kfbase::core::VERTEX_COMPONENT component) const {
      if (!vertex_) return Eigen::MatrixXd::Zero(x.size(), x.size());
      const long bi = getBeginIndex();
      if (bi < 0 || bi + 3 >= x.size()) return Eigen::MatrixXd::Zero(x.size(), x.size());
      Eigen::Vector3d vi;
      vi << calcConversionPoint(x, kfbase::core::VERTEX_X) -
	vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_X),
	calcConversionPoint(x, kfbase::core::VERTEX_Y) -
	vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Y),
	calcConversionPoint(x, kfbase::core::VERTEX_Z) -
	vertex_->calcCartesianCoordinate(x, kfbase::core::VERTEX_Z);
      double norm = vi.norm();
      if (norm < 1e-12) return Eigen::MatrixXd::Zero(x.size(), x.size());
      vi /= norm;
      Eigen::MatrixXd dvi(x.size(), 3);
      dvi << calcDConversionPoint(x, kfbase::core::VERTEX_X) -
	vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_X),
        calcDConversionPoint(x, kfbase::core::VERTEX_Y) -
	vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_Y),
        calcDConversionPoint(x, kfbase::core::VERTEX_Z) -
	vertex_->calcDCartesianCoordinate(x, kfbase::core::VERTEX_Z);
      dvi /= norm;
      const Eigen::VectorXd tv = dvi * vi;
      const int idx = int(component);
      std::vector<Eigen::MatrixXd> d2vi = {
	calcD2ConversionPoint(x, kfbase::core::VERTEX_X) -
	vertex_->calcD2CartesianCoordinate(x, kfbase::core::VERTEX_X),
	calcD2ConversionPoint(x, kfbase::core::VERTEX_Y) -
	vertex_->calcD2CartesianCoordinate(x, kfbase::core::VERTEX_Y),
	calcD2ConversionPoint(x, kfbase::core::VERTEX_Z) -
	vertex_->calcD2CartesianCoordinate(x, kfbase::core::VERTEX_Z)
      };
      for (auto& m : d2vi) m /= norm;
      Eigen::MatrixXd result = d2vi[idx];
      result -= dvi.col(idx) * tv.transpose() + tv * dvi.col(idx).transpose();
      result -= vi(idx) * (vi(0) * d2vi[0] + vi(1) * d2vi[1] + vi(2) * d2vi[2] + dvi * dvi.transpose());
      result += 3. * vi(idx) * tv * tv.transpose();
      return result;
    }

    // ----------------------------------------------------------------------
    // Momentum components (identical to Photon)
    // ----------------------------------------------------------------------
    double BGOLogNormalPhoton::calcOutputMomentumComponent(const Eigen::VectorXd& x,
							   kfbase::core::MOMENT_COMPONENT component) const {
      const long bi = getBeginIndex();
      if (bi < 0 || bi + 3 >= x.size()) return 0.;
      switch (component) {
      case kfbase::core::MOMENT_X: return x(bi) * calcDirection(x, kfbase::core::VERTEX_X);
      case kfbase::core::MOMENT_Y: return x(bi) * calcDirection(x, kfbase::core::VERTEX_Y);
      case kfbase::core::MOMENT_Z: return x(bi) * calcDirection(x, kfbase::core::VERTEX_Z);
      case kfbase::core::MOMENT_E: return x(bi);
      }
      return 0.;
    }

    double BGOLogNormalPhoton::calcInputMomentumComponent(const Eigen::VectorXd&,
							  kfbase::core::MOMENT_COMPONENT) const {
      return 0.;
    }

    Eigen::VectorXd BGOLogNormalPhoton::calcOutputDMomentumComponent(const Eigen::VectorXd& x,
								     kfbase::core::MOMENT_COMPONENT component) const {
      const long bi = getBeginIndex();
      Eigen::VectorXd result = Eigen::VectorXd::Zero(x.size());
      if (bi < 0 || bi + 3 >= x.size()) return result;
      switch (component) {
      case kfbase::core::MOMENT_X:
	result(bi) = calcDirection(x, kfbase::core::VERTEX_X);
	result += x(bi) * calcDDirection(x, kfbase::core::VERTEX_X);
	break;
      case kfbase::core::MOMENT_Y:
	result(bi) = calcDirection(x, kfbase::core::VERTEX_Y);
	result += x(bi) * calcDDirection(x, kfbase::core::VERTEX_Y);
	break;
      case kfbase::core::MOMENT_Z:
	result(bi) = calcDirection(x, kfbase::core::VERTEX_Z);
	result += x(bi) * calcDDirection(x, kfbase::core::VERTEX_Z);
	break;
      case kfbase::core::MOMENT_E:
	result(bi) = 1.;
	break;
      }
      return result;
    }

    Eigen::VectorXd BGOLogNormalPhoton::calcInputDMomentumComponent(const Eigen::VectorXd& x,
								    kfbase::core::MOMENT_COMPONENT) const {
      return Eigen::VectorXd::Zero(x.size());
    }

    Eigen::MatrixXd BGOLogNormalPhoton::calcOutputD2MomentumComponent(const Eigen::VectorXd& x,
								      kfbase::core::MOMENT_COMPONENT component) const {
      const long bi = getBeginIndex();
      Eigen::MatrixXd result = Eigen::MatrixXd::Zero(x.size(), x.size());
      if (bi < 0 || bi + 3 >= x.size()) return result;
      switch (component) {
      case kfbase::core::MOMENT_X:
	result.col(bi) = calcDDirection(x, kfbase::core::VERTEX_X);
	result.row(bi) = result.col(bi).transpose();
	result += x(bi) * calcD2Direction(x, kfbase::core::VERTEX_X);
	break;
      case kfbase::core::MOMENT_Y:
	result.col(bi) = calcDDirection(x, kfbase::core::VERTEX_Y);
	result.row(bi) = result.col(bi).transpose();
	result += x(bi) * calcD2Direction(x, kfbase::core::VERTEX_Y);
	break;
      case kfbase::core::MOMENT_Z:
	result.col(bi) = calcDDirection(x, kfbase::core::VERTEX_Z);
	result.row(bi) = result.col(bi).transpose();
	result += x(bi) * calcD2Direction(x, kfbase::core::VERTEX_Z);
	break;
      case kfbase::core::MOMENT_E:
	break;
      }
      return result;
    }

    Eigen::MatrixXd BGOLogNormalPhoton::calcInputD2MomentumComponent(const Eigen::VectorXd& x,
								     kfbase::core::MOMENT_COMPONENT) const {
      return Eigen::MatrixXd::Zero(x.size(), x.size());
    }

    // ----------------------------------------------------------------------
    // Calibration parameterisation (now with correct normFactor)
    // ----------------------------------------------------------------------
    double BGOLogNormalPhoton::normFactor(double E_var_GeV) const {
      // E_var_GeV in GeV; convert to MeV for calibration constants
      double E_mev = E_var_GeV * 1000.0;
      if (season_ == "11-13") {
	if (E_mev < 100.0)
	  return 1.0278 * (1.0 - std::exp(-904.72 / 341.1));
	else
	  return 1.0278 * (1.0 - std::exp(-(E_mev + 804.72) / 341.1));
      } else if (season_ == "17-24") {
	return 1.03817 - 0.020413 * std::pow(std::abs((E_mev - 1149.41) / 641.335), 3);
      } else {
	// "24-30" or default
	return 1.0;
      }
    }

    double BGOLogNormalPhoton::relResolution(double E_var_GeV) const {
      // Returns sigma/E (dimensionless)
      double a0, a1, a2;
      if (endcap_ == 0) {
	a0 = 0.58899;
	a1 = 1.76457;
	a2 = 3.20554;
      } else {
	a0 = 0.488742;
	a1 = 2.11692;
	a2 = 3.09821;
      }
      double res_percent = std::sqrt( (a0/E_var_GeV)*(a0/E_var_GeV) +
				      (a1/std::sqrt(E_var_GeV))*(a1/std::sqrt(E_var_GeV)) +
				      a2*a2 );
      return res_percent / 100.0;
    }

    double BGOLogNormalPhoton::asymmetry(double /*E_var_GeV*/) const {
      // Constant asymmetry (average from Dubrovin's fits)
      return -0.18;
    }

    // ----------------------------------------------------------------------
    // Target function – overridden with correct log-likelihood
    // ----------------------------------------------------------------------
    double BGOLogNormalPhoton::f(const Eigen::VectorXd& x, bool recalc) const {
      const long bi = getBeginIndex();
      if (bi < 0 || bi + 3 >= x.size()) {
	return 1e10;
      }

      const Eigen::VectorXd& init = getInitialParameters();
      const Eigen::MatrixXd& invCov = getInverseCovarianceMatrix();
      if (init.size() != 4 || invCov.rows() != 4 || invCov.cols() != 4) {
	return 1e10;
      }

      // Gaussian part for R, phi, z (indices 1..3)
      Eigen::VectorXd dx(4);
      for (int i = 0; i < 4; ++i) dx(i) = x(bi + i) - init(i);

      double gauss = 0.0;
      for (int i = 1; i < 4; ++i)
	for (int j = 1; j < 4; ++j)
	  gauss += dx(i) * invCov(i,j) * dx(j);

      // Log‑normal term for energy (index 0)
      double E_var = x(bi);
      double mu = normFactor(E_var) * E_var;        // most probable energy [GeV]
      double sigma = relResolution(E_var) * E_var;  // absolute sigma [GeV]
      double a = asymmetry(E_var);                  // asymmetry parameter

      if (sigma < 1e-12) sigma = 1e-12;
      const double xi = std::sqrt(std::log(4.0));

      // Gaussian limit when asymmetry is negligible
      if (std::abs(a) < 1e-10) {
	double diff = E_meas_ - mu;
	// Correct log-likelihood: χ² + 2*log(sigma)
	double energyTerm = diff * diff / (sigma * sigma) + 2.0 * std::log(sigma);
	return gauss + energyTerm;
      }

      double f_ = std::sinh(a * xi) / (a * xi);
      double x_ = f_ * (E_meas_ - mu) / sigma;

      double energyTerm = 0.0;
      double arg = 1.0 + x_ * a;

      if (arg > 0.0) {
	double logVal = std::log(arg) / a;
	// Full log-likelihood: (logVal^2 + a^2) + 2*log(sigma) + const
	energyTerm = logVal * logVal + a * a + 2.0 * std::log(sigma);
      } else {
	// Smooth penalty outside domain – still include sigma log term
	double delta = x_ * a + 1.0;
	energyTerm = 10.0 * delta * delta + 2.0 * std::log(sigma);
      }

      return gauss + energyTerm;
    }

  } // namespace core
} // namespace kfcmd
