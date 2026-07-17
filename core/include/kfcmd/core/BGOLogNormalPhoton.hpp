#ifndef _KFCMD_BGOLOGNORMALPHOTON_HPP_
#define _KFCMD_BGOLOGNORMALPHOTON_HPP_

#include <kfbase/core/Particle.hpp>
#include <kfbase/core/Vertex.hpp>
#include <string>

namespace kfcmd {
  namespace core {

    class BGOLogNormalPhoton : public kfbase::core::Particle {
    public:
      explicit BGOLogNormalPhoton(const std::string& name);
      virtual ~BGOLogNormalPhoton();

      void setMeasuredEnergy(double e) { E_meas_ = e; }
      void setEndcap(int ec) { endcap_ = ec; }
      void setSeason(const std::string& s) { season_ = s; }
      void setOutputVertex(kfbase::core::Vertex* vertex) { vertex_ = vertex; }

      // Override target function and derivatives with analytical expressions
      virtual double f(const Eigen::VectorXd& x, bool recalc = false) const override;
      virtual Eigen::VectorXd df(const Eigen::VectorXd& x, bool recalc = false) const override;
      virtual Eigen::MatrixXd d2f(const Eigen::VectorXd& x, bool recalc = false) const override;

      // Mandatory Particle interface (identical to Photon)
      virtual double calcOutputMomentumComponent(const Eigen::VectorXd&,
						 kfbase::core::MOMENT_COMPONENT) const override final;
      virtual double calcInputMomentumComponent(const Eigen::VectorXd&,
						kfbase::core::MOMENT_COMPONENT) const override final;
      virtual Eigen::VectorXd calcOutputDMomentumComponent(const Eigen::VectorXd&,
							   kfbase::core::MOMENT_COMPONENT) const override final;
      virtual Eigen::VectorXd calcInputDMomentumComponent(const Eigen::VectorXd&,
							  kfbase::core::MOMENT_COMPONENT) const override final;
      virtual Eigen::MatrixXd calcOutputD2MomentumComponent(const Eigen::VectorXd&,
							    kfbase::core::MOMENT_COMPONENT) const override final;
      virtual Eigen::MatrixXd calcInputD2MomentumComponent(const Eigen::VectorXd&,
							   kfbase::core::MOMENT_COMPONENT) const override final;

    private:
      // Helper methods (same as in Photon)
      double calcConversionPoint(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const;
      Eigen::VectorXd calcDConversionPoint(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const;
      Eigen::MatrixXd calcD2ConversionPoint(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const;
      double calcDirection(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const;
      Eigen::VectorXd calcDDirection(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const;
      Eigen::MatrixXd calcD2Direction(const Eigen::VectorXd&, kfbase::core::VERTEX_COMPONENT) const;

      // Calibration parameterisation
      double relResolution(double E) const;
      double asymmetry(double E) const;
      double normFactor(double E) const; // always 1.0 now

      // Log-normal function and its derivatives
      double logNormalTerm(double E) const;          // L(E) value
      double dLogNormalTerm(double E) const;         // dL/dE
      double d2LogNormalTerm(double E) const;        // d2L/dE2

      // Helper for f0 and derivatives
      double f0(double a) const;
      double df0da(double a) const;
      double d2f0da2(double a) const;

      kfbase::core::Vertex* vertex_;
      double E_meas_;      // measured energy [GeV]
      int    endcap_;      // 0 (forward) or 1 (backward)
      std::string season_; // "11-13", "17-24", "24-30"
    };

  } // namespace core
} // namespace kfcmd

#endif
