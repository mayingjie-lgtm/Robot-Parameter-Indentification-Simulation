// Offline test fixture only: RNEA truth for two independent analytic trajectories.
// This executable does not contain any hardware control or SDK code.
#include "rebot_pinocchio_dynamics.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::runtime_error("usage: rebot_reported_effort_fixture <new-directory>");
    const std::filesystem::path output(argv[1]);
    if (std::filesystem::exists(output)) throw std::runtime_error("fixture directory already exists");
    std::filesystem::create_directories(output);
    rebot_dynamics::ReBotPinocchioDynamics dynamics(
        std::filesystem::path(PROJECT_ROOT_DIR) / "rebot_dm/rebot_dm.urdf", {0.05,0.05});
    for (int split = 0; split < 2; ++split) {
      std::ofstream file(output / (split ? "B.truth.csv" : "A.truth.csv"));
      file << std::setprecision(17) << "time";
      for (const auto *prefix : {"q", "qd", "qdd", "tau"})
        for (int j = 0; j < 6; ++j) file << ',' << prefix << j;
      file << '\n';
      for (int i = 0; i <= 3000; ++i) {
        const double t = i * 0.01;
        Eigen::VectorXd q(6), qd(6), qdd(6);
        for (int j = 0; j < 6; ++j) {
          q(j) = j == 1 || j == 2 ? -1.0 : 0.0;
          qd(j) = qdd(j) = 0.0;
          for (int harmonic = 1; harmonic <= 3; ++harmonic) {
            const double w = 2 * M_PI * (0.025 * harmonic + 0.009*j + 0.007*split);
            const double phase = 0.43*j + 0.79*split + 0.3*harmonic;
            const double a = 0.20 / harmonic;
            q(j) += a * std::sin(w*t+phase);
            qd(j) += a*w * std::cos(w*t+phase);
            qdd(j) -= a*w*w * std::sin(w*t+phase);
          }
        }
        Eigen::VectorXd tau = dynamics.inverseDynamics(q,qd,qdd);
        for (int j = 0; j < 6; ++j)
          tau(j) += (0.02+0.002*j)*qdd(j) + (0.03+0.003*j)*qd(j)
                  + (0.04+0.005*j)*(qd(j)>0 ? 1.0 : -1.0);
        file << t;
        for (const auto *v : {&q,&qd,&qdd,&tau})
          for (int j=0; j<6; ++j) file << ',' << (*v)(j);
        file << '\n';
      }
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
