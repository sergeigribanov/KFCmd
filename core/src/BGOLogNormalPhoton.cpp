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
    // Calibration parameterisation
    // ----------------------------------------------------------------------
    double BGOLogNormalPhoton::normFactor(double /*E*/) const {
      return 1.0;
    }

    double BGOLogNormalPhoton::relResolution(double E) const {
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
      double res_percent = std::sqrt( (a0/E)*(a0/E) +
				      (a1/std::sqrt(E))*(a1/std::sqrt(E)) +
				      a2*a2 );
      return res_percent / 100.0;
    }

    double BGOLogNormalPhoton::asymmetry(double E) const {
      double E_mev = E * 1000.0;
      double p0, p1;
      if (endcap_ == 0) {
	p0 = -0.184602;
	p1 = -7.4081e-05;
      } else {
	p0 = -0.175625;
	p1 = -8.59385e-05;
      }
      return p0 + p1 * E_mev;
    }

    // ----------------------------------------------------------------------
    // f0 = sinh(a*xi)/(a*xi) and derivatives
    // ----------------------------------------------------------------------
    double BGOLogNormalPhoton::f0(double a) const {
      const double xi = std::sqrt(std::log(4.0));
      if (std::abs(a) < 1e-12) return 1.0;
      double ax = a * xi;
      return std::sinh(ax) / ax;
    }

    double BGOLogNormalPhoton::df0da(double a) const {
      const double xi = std::sqrt(std::log(4.0));
      if (std::abs(a) < 1e-12) return 0.0;
      double ax = a * xi;
      double sh = std::sinh(ax);
      double ch = std::cosh(ax);
      return xi * (ax * ch - sh) / (ax * ax);
    }

    double BGOLogNormalPhoton::d2f0da2(double a) const {
      const double xi = std::sqrt(std::log(4.0));
      if (std::abs(a) < 1e-12) return xi * xi / 3.0;
      double ax = a * xi;
      double sh = std::sinh(ax);
      double ch = std::cosh(ax);
      return xi * xi * (ax * ax * sh - 2.0 * ax * ch + 2.0 * sh) / (ax * ax * ax);
    }

    // ----------------------------------------------------------------------
    // Analytical derivatives of logNormalTerm
    // ----------------------------------------------------------------------
    double BGOLogNormalPhoton::logNormalTerm(double E) const {
      double s = relResolution(E) * E;
      double a = asymmetry(E);
      double mu = E;
      double m = E_meas_;

      if (s < 1e-12) s = 1e-12;

      if (std::abs(a) < 1e-10) {
	double diff = m - mu;
	return diff * diff / (s * s) + 2.0 * std::log(s);
      }

      double f = f0(a);
      double x0 = f * (m - mu) / s;
      double arg = 1.0 + x0 * a;
      if (arg > 0.0) {
	double logVal = std::log(arg) / a;
	return logVal * logVal + a * a + 2.0 * std::log(s) - 2.0 * std::log(f);
      } else {
	double delta = x0 * a + 1.0;
	return 10.0 * delta * delta + 2.0 * std::log(s) - 2.0 * std::log(f);
      }
    }

    // ----------------------------------------------------------------------
    // Analytical derivatives of logNormalTerm (first and second)
    // ----------------------------------------------------------------------
    double BGOLogNormalPhoton::dLogNormalTerm(double E) const {
      double m = E_meas_;
      double sigma = relResolution(E) * E;
      double a = asymmetry(E);

      if (sigma < 1e-12) sigma = 1e-12;

      // Derivatives of a (linear)
      double a_prime, a_dprime;
      if (endcap_ == 0) {
	a_prime = -7.4081e-05 * 1000.0; // da/dE (GeV^-1)
	a_dprime = 0.0;
      } else {
	a_prime = -8.59385e-05 * 1000.0;
	a_dprime = 0.0;
      }

      // Resolution and its derivatives (analytical)
      double a0, a1, a2;
      if (endcap_ == 0) {
	a0 = 0.58899; a1 = 1.76457; a2 = 3.20554;
      } else {
	a0 = 0.488742; a1 = 2.11692; a2 = 3.09821;
      }
      double E2 = E*E, E3 = E2*E, E4 = E2*E2;
      double u = (a0*a0)/E2 + (a1*a1)/E + a2*a2;
      double u_prime = -2.0*a0*a0/E3 - a1*a1/E2;
      double u_dprime = 6.0*a0*a0/E4 + 2.0*a1*a1/E3;
      double sqrt_u = std::sqrt(u);
      double q_res = 0.01 * sqrt_u;
      double q_prime_res = 0.005 * u_prime / sqrt_u;
      double q_dprime_res = 0.005 * (u_dprime - u_prime*u_prime/(2.0*u)) / sqrt_u;

      double sigma_prime = q_res + E * q_prime_res;
      double sigma_dprime = 2.0 * q_prime_res + E * q_dprime_res;

      // f0 and derivatives
      double f = f0(a);
      double df_da = df0da(a);
      double d2f_da2 = d2f0da2(a);
      double f_prime = df_da * a_prime;
      double f_dprime = d2f_da2 * a_prime * a_prime + df_da * a_dprime;

      // x0 = f * (m - E) / sigma
      double x0 = f * (m - E) / sigma;
      double x0_prime = (f_prime * (m - E) - f) / sigma - f * (m - E) * sigma_prime / (sigma * sigma);
      double x0_dprime;
      double A = f, B = m - E, C = 1.0/sigma;
      double A_p = f_prime, B_p = -1.0, C_p = -sigma_prime/(sigma*sigma);
      double A_pp = f_dprime, B_pp = 0.0, C_pp = -sigma_dprime/(sigma*sigma) + 2.0*sigma_prime*sigma_prime/(sigma*sigma*sigma);
      x0_dprime = A_pp * B * C
	+ 2.0 * A_p * B_p * C
	+ 2.0 * A_p * B * C_p
	+ A * B_pp * C
	+ 2.0 * A * B_p * C_p
	+ A * B * C_pp;

      // t = x0 * a
      double t = x0 * a;
      double t_prime = x0_prime * a + x0 * a_prime;
      double t_dprime = x0_dprime * a + 2.0 * x0_prime * a_prime + x0 * a_dprime;

      // logVal = ln(1+t)/a
      double arg = 1.0 + t;
      double logVal = std::log(arg) / a;
      double logVal_prime = (t_prime / arg * a - std::log(arg) * a_prime) / (a * a);
      double N = a * t_prime / arg - a_prime * std::log(arg);
      double N_prime = (a_prime * t_prime + a * t_dprime) / arg
	- a * t_prime * t_prime / (arg * arg)
	- (a_dprime * std::log(arg) + a_prime * t_prime / arg);
      double logVal_dprime = (N_prime * a * a - N * 2.0 * a * a_prime) / (a * a * a * a);

      // L' = dL/dE
      double L_prime = 2.0 * logVal * logVal_prime
	+ 2.0 * a * a_prime
	+ 2.0 * sigma_prime / sigma
	- 2.0 * f_prime / f;

      return L_prime;
    }

    double BGOLogNormalPhoton::d2LogNormalTerm(double E) const {
      double m = E_meas_;
      double sigma = relResolution(E) * E;
      double a = asymmetry(E);

      if (sigma < 1e-12) sigma = 1e-12;

      // Derivatives of a
      double a_prime, a_dprime;
      if (endcap_ == 0) {
	a_prime = -7.4081e-05 * 1000.0;
	a_dprime = 0.0;
      } else {
	a_prime = -8.59385e-05 * 1000.0;
	a_dprime = 0.0;
      }

      // Resolution derivatives
      double a0, a1, a2;
      if (endcap_ == 0) {
	a0 = 0.58899; a1 = 1.76457; a2 = 3.20554;
      } else {
	a0 = 0.488742; a1 = 2.11692; a2 = 3.09821;
      }
      double E2 = E*E, E3 = E2*E, E4 = E2*E2;
      double u = (a0*a0)/E2 + (a1*a1)/E + a2*a2;
      double u_prime = -2.0*a0*a0/E3 - a1*a1/E2;
      double u_dprime = 6.0*a0*a0/E4 + 2.0*a1*a1/E3;
      double sqrt_u = std::sqrt(u);
      double q_res = 0.01 * sqrt_u;
      double q_prime_res = 0.005 * u_prime / sqrt_u;
      double q_dprime_res = 0.005 * (u_dprime - u_prime*u_prime/(2.0*u)) / sqrt_u;

      double sigma_prime = q_res + E * q_prime_res;
      double sigma_dprime = 2.0 * q_prime_res + E * q_dprime_res;

      // f0 and derivatives
      double f = f0(a);
      double df_da = df0da(a);
      double d2f_da2 = d2f0da2(a);
      double f_prime = df_da * a_prime;
      double f_dprime = d2f_da2 * a_prime * a_prime + df_da * a_dprime;

      // x0 and derivatives
      double x0 = f * (m - E) / sigma;
      double x0_prime = (f_prime * (m - E) - f) / sigma - f * (m - E) * sigma_prime / (sigma * sigma);
      double x0_dprime;
      double A = f, B = m - E, C = 1.0/sigma;
      double A_p = f_prime, B_p = -1.0, C_p = -sigma_prime/(sigma*sigma);
      double A_pp = f_dprime, B_pp = 0.0, C_pp = -sigma_dprime/(sigma*sigma) + 2.0*sigma_prime*sigma_prime/(sigma*sigma*sigma);
      x0_dprime = A_pp * B * C
	+ 2.0 * A_p * B_p * C
	+ 2.0 * A_p * B * C_p
	+ A * B_pp * C
	+ 2.0 * A * B_p * C_p
	+ A * B * C_pp;

      // t = x0 * a
      double t = x0 * a;
      double t_prime = x0_prime * a + x0 * a_prime;
      double t_dprime = x0_dprime * a + 2.0 * x0_prime * a_prime + x0 * a_dprime;

      // logVal and derivatives
      double arg = 1.0 + t;
      double logVal = std::log(arg) / a;
      double logVal_prime = (t_prime / arg * a - std::log(arg) * a_prime) / (a * a);
      double N = a * t_prime / arg - a_prime * std::log(arg);
      double N_prime = (a_prime * t_prime + a * t_dprime) / arg
	- a * t_prime * t_prime / (arg * arg)
	- (a_dprime * std::log(arg) + a_prime * t_prime / arg);
      double logVal_dprime = (N_prime * a * a - N * 2.0 * a * a_prime) / (a * a * a * a);

      // L'' = d2L/dE2
      double L_dprime = 2.0 * (logVal_prime * logVal_prime + logVal * logVal_dprime)
	+ 2.0 * (a_prime * a_prime + a * a_dprime)
	+ 2.0 * (sigma_dprime * sigma - sigma_prime * sigma_prime) / (sigma * sigma)
	- 2.0 * (f_dprime * f - f_prime * f_prime) / (f * f);

      return L_dprime;
    }

    // ----------------------------------------------------------------------
    // Target function and derivatives (overridden)
    // ----------------------------------------------------------------------
    double BGOLogNormalPhoton::f(const Eigen::VectorXd& x, bool recalc) const {
      const long bi = getBeginIndex();
      if (bi < 0 || bi + 3 >= x.size()) return 1e10;

      const Eigen::VectorXd& init = getInitialParameters();
      const Eigen::MatrixXd& invCov = getInverseCovarianceMatrix();
      if (init.size() != 4 || invCov.rows() != 4 || invCov.cols() != 4) return 1e10;

      // Gaussian part for R, phi, z (indices 1..3)
      Eigen::VectorXd dx(4);
      for (int i = 0; i < 4; ++i) dx(i) = x(bi + i) - init(i);

      double gauss = 0.0;
      for (int i = 1; i < 4; ++i)
	for (int j = 1; j < 4; ++j)
	  gauss += dx(i) * invCov(i,j) * dx(j);

      double E = x(bi);
      double lnpart = logNormalTerm(E);

      return gauss + lnpart;
    }

    Eigen::VectorXd BGOLogNormalPhoton::df(const Eigen::VectorXd& x, bool recalc) const {
      const long bi = getBeginIndex();
      const long n = x.size();
      Eigen::VectorXd grad = Eigen::VectorXd::Zero(n);

      if (bi < 0 || bi + 3 >= n) return grad;

      const Eigen::VectorXd& init = getInitialParameters();
      const Eigen::MatrixXd& invCov = getInverseCovarianceMatrix();
      if (init.size() != 4 || invCov.rows() != 4 || invCov.cols() != 4) return grad;

      // Gaussian part for all parameters: grad_gauss = 2 * invCov * (x - init)
      Eigen::VectorXd dx(4);
      for (int i = 0; i < 4; ++i) dx(i) = x(bi + i) - init(i);
  
      for (int i = 0; i < 4; ++i) {
	for (int j = 0; j < 4; ++j) {
	  grad(bi + i) += 2.0 * invCov(i,j) * dx(j);
	}
      }

      // Replace energy component with analytical derivative
      double E = x(bi);
      grad(bi) = dLogNormalTerm(E);

      return grad;
    }

    Eigen::MatrixXd BGOLogNormalPhoton::d2f(const Eigen::VectorXd& x, bool recalc) const {
      const long bi = getBeginIndex();
      const long n = x.size();
      Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(n, n);

      if (bi < 0 || bi + 3 >= n) return hess;

      const Eigen::VectorXd& init = getInitialParameters();
      const Eigen::MatrixXd& invCov = getInverseCovarianceMatrix();
      if (init.size() != 4 || invCov.rows() != 4 || invCov.cols() != 4) return hess;

      // Gaussian part for all parameters: hess_gauss = 2 * invCov
      for (int i = 0; i < 4; ++i)
	for (int j = 0; j < 4; ++j)
	  hess(bi + i, bi + j) = 2.0 * invCov(i,j);

      // Replace energy-energy component with analytical second derivative
      double E = x(bi);
      hess(bi, bi) = d2LogNormalTerm(E);

      // Zero out mixed derivatives with energy
      for (int i = 1; i < 4; ++i) {
	hess(bi, bi + i) = 0.0;
	hess(bi + i, bi) = 0.0;
      }

      return hess;
    }

  } // namespace core
} // namespace kfcmd
