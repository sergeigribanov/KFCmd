#ifndef _KFCMD_BGOLOGNORMALPHOTON_HPP_
#define _KFCMD_BGOLOGNORMALPHOTON_HPP_

#include <kfbase/core/Particle.hpp>
#include <kfbase/core/Vertex.hpp>
#include <string>

namespace kfcmd {
  namespace core {

    /**
     * Photon with log‑normal energy PDF for BGO endcap.
     * Parameters: (E, R_c, phi_c, z_c) – same as Photon.
     * The target function f replaces the Gaussian energy penalty with
     * -2*log( f_lognormal(E_meas; μ(E_var), σ(E_var), a(E_var)) )
     * where μ, σ, a are parameterised from BGO calibration.
     */
    class BGOLogNormalPhoton : public kfbase::core::Particle {
    public:
      explicit BGOLogNormalPhoton(const std::string& name);
      virtual ~BGOLogNormalPhoton();

      // Setters for measured parameters and calibration context
      void setMeasuredEnergy(double e) { E_meas_ = e; }
      void setEndcap(int ec) { endcap_ = ec; }
      void setSeason(const std::string& s) { season_ = s; }
      void setOutputVertex(kfbase::core::Vertex* vertex) { vertex_ = vertex; }

      // Override target function
      virtual double f(const Eigen::VectorXd& x, bool recalc = false) const override;

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
      double normFactor(double E_var_GeV) const;
      double relResolution(double E_var_GeV) const;
      double asymmetry(double E_var_GeV) const;

      kfbase::core::Vertex* vertex_;
      double E_meas_;      // measured energy [GeV]
      int    endcap_;      // 0 (forward) or 1 (backward)
      std::string season_; // "11-13", "17-24", "24-30"
    };

  } // namespace core
} // namespace kfcmd

#endif
