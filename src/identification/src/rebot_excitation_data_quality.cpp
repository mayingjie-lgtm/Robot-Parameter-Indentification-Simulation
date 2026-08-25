#include "identification/data_loader.hpp"
#include "mujoco_regressor.hpp"
#include "rebot_pinocchio_regressor.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kRelativeRankThreshold = 1e-6;
constexpr double kStructuralZeroTolerance = 1e-14;
constexpr double kNearZeroColumnRelativeTolerance = 1e-10;

struct RankResult {
  Eigen::Index raw_columns{0};
  Eigen::Index rank{0};
  double effective_condition{std::numeric_limits<double>::infinity()};
  Eigen::VectorXd singular_values;
  std::vector<Eigen::Index> structural_zero_columns;
  std::vector<Eigen::Index> near_zero_columns;
};

/**
 * Compute the column-scaled SVD gate used by the trusted reBot regressor baseline.
 *
 * @param regressor reBot Pinocchio augmented regressor.
 * @param q Actual trajectory positions arranged as 6xN.
 * @param qd Actual trajectory velocities arranged as 6xN.
 * @param qdd Actual MuJoCo accelerations arranged as 6xN.
 * @param flags Requested 60/72/78-column parameter layout.
 * @param label Human-readable layout label for diagnostics.
 * @return Rank and singular-value diagnostics without fitting parameters.
 */
RankResult analyzeRank(rebot_dynamics::ReBotPinocchioRegressor &regressor,
                       const Eigen::MatrixXd &q, const Eigen::MatrixXd &qd,
                       const Eigen::MatrixXd &qdd,
                       mujoco_dynamics::MuJoCoParamFlags flags,
                       const std::string &label) {
  const Eigen::MatrixXd observation =
      regressor.computeObservationMatrix(q, qd, qdd, flags);
  Eigen::MatrixXd scaled = observation;
  Eigen::VectorXd column_norms(observation.cols());
  double max_column_norm = 0.0;
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    const double norm = observation.col(column).norm();
    column_norms(column) = norm;
    max_column_norm = std::max(max_column_norm, norm);
  }

  RankResult result;
  result.raw_columns = observation.cols();
  const double near_zero_tolerance =
      kNearZeroColumnRelativeTolerance * max_column_norm;
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    if (column_norms(column) <= kStructuralZeroTolerance) {
      result.structural_zero_columns.push_back(column);
      continue;
    }
    if (column_norms(column) <= near_zero_tolerance) {
      result.near_zero_columns.push_back(column);
    }
    scaled.col(column) /= column_norms(column);
  }

  // The actual trajectory has 180k torque rows. Compute singular values from
  // the small column Gram matrix so every sample participates without a costly
  // tall-matrix SVD or any observation subsampling.
  const Eigen::MatrixXd gram = scaled.transpose() * scaled;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(gram);
  if (eigen_solver.info() != Eigen::Success) {
    throw std::runtime_error("failed to eigendecompose scaled observation Gram matrix");
  }
  const Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
  result.singular_values.resize(eigenvalues.size());
  for (Eigen::Index index = 0; index < eigenvalues.size(); ++index) {
    const double value = eigenvalues(eigenvalues.size() - 1 - index);
    result.singular_values(index) = std::sqrt(std::max(0.0, value));
  }
  const double sigma_max = result.singular_values(0);
  const double absolute_threshold = kRelativeRankThreshold * sigma_max;
  result.rank = (result.singular_values.array() > absolute_threshold).count();
  if (result.rank > 0) {
    result.effective_condition = sigma_max / result.singular_values(result.rank - 1);
  }

  const auto names = regressor.getParameterNames(flags);
  std::cout << label << " raw_columns=" << result.raw_columns
            << " numerical_rank=" << result.rank
            << " relative_rank_threshold=" << kRelativeRankThreshold
            << " effective_condition=" << std::setprecision(12)
            << result.effective_condition << "\n";

  std::cout << label << " structural_zero_columns=";
  if (result.structural_zero_columns.empty()) {
    std::cout << "none";
  } else {
    for (const Eigen::Index column : result.structural_zero_columns) {
      std::cout << column << "(" << names.at(static_cast<std::size_t>(column)) << ") ";
    }
  }
  std::cout << "\n";

  std::cout << label << " near_zero_columns=";
  if (result.near_zero_columns.empty()) {
    std::cout << "none";
  } else {
    for (const Eigen::Index column : result.near_zero_columns) {
      std::cout << column << "(" << names.at(static_cast<std::size_t>(column)) << ") ";
    }
  }
  std::cout << "\n";
  std::cout << label << " near_zero_singular_directions="
            << (result.raw_columns - result.rank) << "\n";

  std::cout << label << " singular_values=[";
  for (Eigen::Index index = 0; index < result.singular_values.size(); ++index) {
    std::cout << std::scientific << std::setprecision(8) << result.singular_values(index);
    if (index + 1 < result.singular_values.size()) {
      std::cout << ", ";
    }
  }
  std::cout << "]\n";
  return result;
}

/** Return the frozen reBot regressor rank expected for one raw-column layout. */
Eigen::Index expectedRank(Eigen::Index raw_columns) {
  if (raw_columns == 60) return 36;
  if (raw_columns == 72) return 46;
  if (raw_columns == 78) return 52;
  throw std::runtime_error("unexpected reBot excitation raw-column count");
}

} // namespace

/** Build actual-trajectory observation matrices and enforce the frozen reBot rank. */
int main(int argc, char **argv) {
  try {
    if (argc != 2) {
      std::cerr << "Usage: rebot_excitation_data_quality <trajectory.csv>\n";
      return 2;
    }

    const ExperimentData data = DataLoader::loadCSV(argv[1], 6);
    if (data.n_samples == 0 || data.n_dof != 6) {
      throw std::runtime_error("excitation CSV must contain six-DoF samples");
    }
    if (!data.q.allFinite() || !data.qd.allFinite() || !data.qdd.allFinite()) {
      throw std::runtime_error("excitation q/qd/qdd contains non-finite values");
    }

    const Eigen::MatrixXd q = data.q.transpose();
    const Eigen::MatrixXd qd = data.qd.transpose();
    const Eigen::MatrixXd qdd = data.qdd.transpose();
    rebot_dynamics::ReBotPinocchioRegressor regressor(
        std::filesystem::path(PROJECT_ROOT_DIR) / "rebot_dm" / "rebot_dm.urdf");

    std::cout << "trajectory_csv=" << argv[1] << " samples=" << data.n_samples
              << " relative_rank_threshold=" << kRelativeRankThreshold << "\n";

    const RankResult rigid = analyzeRank(
        regressor, q, qd, qdd, mujoco_dynamics::MuJoCoParamFlags::NONE, "Rank 60");
    const RankResult augmented = analyzeRank(
        regressor, q, qd, qdd, mujoco_dynamics::MuJoCoParamFlags::ALL, "Rank 72");
    const RankResult friction = analyzeRank(
        regressor, q, qd, qdd,
        mujoco_dynamics::MuJoCoParamFlags::ALL_WITH_FRICTION, "Rank 78");

    bool passed = true;
    for (const RankResult *result : {&rigid, &augmented, &friction}) {
      const Eigen::Index expected = expectedRank(result->raw_columns);
      if (result->rank != expected) {
        std::cerr << "rank gate failed for " << result->raw_columns
                  << " columns: expected " << expected << ", got "
                  << result->rank << "\n";
        passed = false;
      }
    }

    if (!passed) {
      std::cerr << "[FAIL] reBot-DM actual-trajectory rank gate\n";
      return 1;
    }
    std::cout << "[PASS] reBot-DM actual-trajectory rank gate\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "reBot excitation data-quality gate failed: " << error.what() << "\n";
    return 1;
  }
}
