#include "trajectory/fourier_trajectory.hpp"

#include <Eigen/Dense>

#include <filesystem>
#include <iostream>
#include <random>
#include <stdexcept>
#include <unistd.h>

namespace {

/** Require exact equality because reproducibility is a bitwise contract. */
void requireExact(const Eigen::VectorXd &lhs, const Eigen::VectorXd &rhs,
                  const std::string &message) {
  if (lhs.size() != rhs.size() || !(lhs.array() == rhs.array()).all()) {
    throw std::runtime_error(message);
  }
}

/** Exercise generation, seed separation, CSV round-trip, and replay. */
void runReproducibilityTest() {
  Eigen::VectorXd q0(6);
  q0 << 0.0, 0.8, -0.8, 0.0, 0.0, 0.0;
  trajectory::FourierTrajectory first(6, 5, 30.0, q0);
  trajectory::FourierTrajectory second(6, 5, 30.0, q0);
  trajectory::FourierTrajectory other_seed(6, 5, 30.0, q0);
  std::mt19937 first_rng(20260824U);
  std::mt19937 second_rng(20260824U);
  std::mt19937 other_rng(20260825U);
  first.setRandomCoefficients(0.15, first_rng);
  second.setRandomCoefficients(0.15, second_rng);
  other_seed.setRandomCoefficients(0.15, other_rng);

  requireExact(first.getParameterVector(), second.getParameterVector(),
               "same seed did not produce identical coefficients");
  if ((first.getParameterVector().array() ==
       other_seed.getParameterVector().array()).all()) {
    throw std::runtime_error("different seeds produced identical coefficients");
  }

  const auto path = std::filesystem::temp_directory_path() /
                    ("piper_fourier_" + std::to_string(::getpid()) + ".csv");
  first.saveCoefficients(path);
  trajectory::FourierTrajectory replay(6, 5, 30.0, q0);
  replay.loadCoefficients(path);
  std::filesystem::remove(path);
  requireExact(first.getParameterVector(), replay.getParameterVector(),
               "coefficient CSV did not round-trip exactly");

  for (const double time : {0.0, 0.001, 7.25, 15.0, 29.999}) {
    const auto expected = first.evaluate(time);
    const auto actual = replay.evaluate(time);
    requireExact(expected.q, actual.q, "replayed q differs");
    requireExact(expected.qd, actual.qd, "replayed qd differs");
    requireExact(expected.qdd, actual.qdd, "replayed qdd differs");
  }
}

} // namespace

int main() {
  try {
    runReproducibilityTest();
    std::cout << "Fourier trajectory reproducibility test passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Fourier trajectory reproducibility test failed: "
              << error.what() << "\n";
    return 1;
  }
}
