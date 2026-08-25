/**
 * @file fourier_trajectory.cpp
 * @brief Fourier series trajectory implementation
 */

#include "trajectory/fourier_trajectory.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace trajectory {

FourierTrajectory::FourierTrajectory(const FourierParameters &params)
    : params_(params) {}

FourierTrajectory::FourierTrajectory(std::size_t n_dof, std::size_t n_harmonics,
                                     double period, const Eigen::VectorXd &q0) {
  params_.n_dof = n_dof;
  params_.n_harmonics = n_harmonics;
  params_.omega = 2.0 * M_PI / period;

  if (q0.size() > 0) {
    params_.q0 = q0;
  } else {
    params_.q0 = Eigen::VectorXd::Zero(n_dof);
  }

  params_.A = Eigen::MatrixXd::Zero(n_dof, n_harmonics);
  params_.B = Eigen::MatrixXd::Zero(n_dof, n_harmonics);
}

TrajectoryPoint FourierTrajectory::evaluate(double t) const {
  const std::size_t n = params_.n_dof;
  const std::size_t n_f = params_.n_harmonics;
  const double omega = params_.omega;

  TrajectoryPoint pt;
  pt.time = t;
  pt.q = params_.q0;
  pt.qd = Eigen::VectorXd::Zero(n);
  pt.qdd = Eigen::VectorXd::Zero(n);

  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n_f; ++j) {
      const double A_ij = params_.A(i, j);
      const double B_ij = params_.B(i, j);
      const double j1 = static_cast<double>(j + 1);
      const double omega_j = omega * j1;

      const double cos_wt = std::cos(omega_j * t);
      const double sin_wt = std::sin(omega_j * t);

      // q_i(t) = q0_i + Σ [ A_ij/(ωj) * sin(ωjt) - B_ij/(ωj) * cos(ωjt) ]
      pt.q(i) += A_ij / omega_j * sin_wt - B_ij / omega_j * cos_wt;

      // q̇_i(t) = Σ [ A_ij * cos(ωjt) + B_ij * sin(ωjt) ]
      pt.qd(i) += A_ij * cos_wt + B_ij * sin_wt;

      // q̈_i(t) = ω * Σ [ B_ij * j * cos(ωjt) - A_ij * j * sin(ωjt) ]
      pt.qdd(i) += omega * (B_ij * j1 * cos_wt - A_ij * j1 * sin_wt);
    }
  }

  return pt;
}

void FourierTrajectory::setCoefficients(const Eigen::MatrixXd &A,
                                        const Eigen::MatrixXd &B) {
  params_.A = A;
  params_.B = B;
}

void FourierTrajectory::setParameterVector(const Eigen::VectorXd &x) {
  const std::size_t n = params_.n_dof;
  const std::size_t n_f = params_.n_harmonics;
  const std::size_t half = n * n_f;

  // x = [A_11, A_12, ..., A_1n_f, A_21, ..., A_n_n_f, B_11, ...]
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n_f; ++j) {
      params_.A(i, j) = x(i * n_f + j);
      params_.B(i, j) = x(half + i * n_f + j);
    }
  }
}

Eigen::VectorXd FourierTrajectory::getParameterVector() const {
  const std::size_t n = params_.n_dof;
  const std::size_t n_f = params_.n_harmonics;
  const std::size_t total = 2 * n * n_f;

  Eigen::VectorXd x(total);
  const std::size_t half = n * n_f;

  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n_f; ++j) {
      x(i * n_f + j) = params_.A(i, j);
      x(half + i * n_f + j) = params_.B(i, j);
    }
  }

  return x;
}

void FourierTrajectory::setRandomCoefficients(double amplitude,
                                              std::mt19937 &generator) {
  for (std::size_t i = 0; i < params_.n_dof; ++i) {
    for (std::size_t j = 0; j < params_.n_harmonics; ++j) {
      // The explicit integer-to-double mapping is reproducible across standard
      // library implementations, unlike uniform_real_distribution.
      const double unit_a = static_cast<double>(generator()) /
                            static_cast<double>(generator.max());
      const double unit_b = static_cast<double>(generator()) /
                            static_cast<double>(generator.max());
      params_.A(i, j) = amplitude * (2.0 * unit_a - 1.0);
      params_.B(i, j) = amplitude * (2.0 * unit_b - 1.0);
    }
  }

  applyInitialConditionConstraints();
}

void FourierTrajectory::saveCoefficients(
    const std::filesystem::path &path) const {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Unable to save Fourier coefficients: " +
                             path.string());
  }
  output << "joint,harmonic,omega,q0,A,B\n" << std::setprecision(17);
  for (std::size_t joint = 0; joint < params_.n_dof; ++joint) {
    for (std::size_t harmonic = 0; harmonic < params_.n_harmonics;
         ++harmonic) {
      output << joint << "," << harmonic << "," << params_.omega << ","
             << params_.q0(static_cast<Eigen::Index>(joint)) << ","
             << params_.A(static_cast<Eigen::Index>(joint),
                          static_cast<Eigen::Index>(harmonic))
             << ","
             << params_.B(static_cast<Eigen::Index>(joint),
                          static_cast<Eigen::Index>(harmonic))
             << "\n";
    }
  }
}

void FourierTrajectory::loadCoefficients(
    const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Unable to load Fourier coefficients: " +
                             path.string());
  }
  std::string line;
  if (!std::getline(input, line) || line != "joint,harmonic,omega,q0,A,B") {
    throw std::runtime_error("Invalid Fourier coefficient CSV header");
  }
  std::vector<bool> seen(params_.n_dof * params_.n_harmonics, false);
  std::size_t row = 1;
  while (std::getline(input, line)) {
    ++row;
    std::stringstream stream(line);
    std::vector<std::string> values;
    std::string value;
    while (std::getline(stream, value, ',')) {
      values.push_back(value);
    }
    if (values.size() != 6) {
      throw std::runtime_error("Invalid Fourier coefficient row " +
                               std::to_string(row));
    }
    const std::size_t joint = std::stoul(values[0]);
    const std::size_t harmonic = std::stoul(values[1]);
    if (joint >= params_.n_dof || harmonic >= params_.n_harmonics) {
      throw std::runtime_error("Fourier coefficient index is out of range");
    }
    const double omega = std::stod(values[2]);
    const double q0 = std::stod(values[3]);
    if (omega != params_.omega ||
        q0 != params_.q0(static_cast<Eigen::Index>(joint))) {
      throw std::runtime_error(
          "Fourier coefficient metadata does not match controller config");
    }
    const std::size_t index = joint * params_.n_harmonics + harmonic;
    if (seen[index]) {
      throw std::runtime_error("Duplicate Fourier coefficient row");
    }
    seen[index] = true;
    params_.A(static_cast<Eigen::Index>(joint),
              static_cast<Eigen::Index>(harmonic)) = std::stod(values[4]);
    params_.B(static_cast<Eigen::Index>(joint),
              static_cast<Eigen::Index>(harmonic)) = std::stod(values[5]);
  }
  for (const bool present : seen) {
    if (!present) {
      throw std::runtime_error("Fourier coefficient CSV is incomplete");
    }
  }
}

void FourierTrajectory::applyInitialConditionConstraints() {
  const double omega = params_.omega;

  // At t=0, we need to satisfy:
  // 1. q̇(0) = Σ A_ij = 0
  // 2. q(0) = q0, which requires: Σ B_ij/(ω*(j+1)) = 0
  // 3. q̈(0) = 0, which requires: Σ B_ij * (j+1) = 0
  //
  // Constraints 2 and 3 require TWO degrees of freedom (B_i0 and B_i1)
  // to satisfy two independent linear equations.

  for (std::size_t i = 0; i < params_.n_dof; ++i) {
    // Constraint 1: Σ A_ij = 0 for q̇(0) = 0
    // Use A(i,0) to cancel sum of A(i,1..n-1)
    double sum_A = 0;
    for (std::size_t j = 1; j < params_.n_harmonics; ++j) {
      sum_A += params_.A(i, j);
    }
    params_.A(i, 0) = -sum_A;

    // For constraints 2 and 3, we need to solve a 2x2 linear system
    // using B(i,0) and B(i,1) as free variables.
    //
    // Let S1 = Σ_{j>=2} B_ij/(ω*(j+1))  (contribution from fixed B coefficients)
    // Let S2 = Σ_{j>=2} B_ij * (j+1)     (contribution from fixed B coefficients)
    //
    // System:
    //   B0/(ω*1) + B1/(ω*2) = -S1   => B0 + B1/2 = -S1*ω   ...(eq1)
    //   B0*1 + B1*2 = -S2                                   ...(eq2)
    //
    // Solving:
    //   (eq2) - (eq1): 1.5*B1 = -S2 + S1*ω
    //   B1 = (S1*ω - S2) * (2/3)
    //   B0 = -S1*ω - B1/2

    if (params_.n_harmonics >= 2) {
      double S1 = 0.0; // Σ B_ij / (ω*(j+1)) for j >= 2
      double S2 = 0.0; // Σ B_ij * (j+1) for j >= 2

      for (std::size_t j = 2; j < params_.n_harmonics; ++j) {
        const double j1 = static_cast<double>(j + 1);
        S1 += params_.B(i, j) / (omega * j1);
        S2 += params_.B(i, j) * j1;
      }

      // Solve 2x2 system
      const double B1 = (S1 * omega - S2) * (2.0 / 3.0);
      const double B0 = -S1 * omega - B1 / 2.0;

      params_.B(i, 0) = B0;
      params_.B(i, 1) = B1;
    } else {
      // Only one harmonic: can only satisfy one constraint
      // Prioritize q̈(0) = 0
      params_.B(i, 0) = 0.0;
    }
  }
}

} // namespace trajectory
