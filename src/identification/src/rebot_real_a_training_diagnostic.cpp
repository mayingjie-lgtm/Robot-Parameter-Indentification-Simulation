#include "identification/algorithms.hpp"
#include "identification/data_loader.hpp"
#include "mujoco_regressor.hpp"
#include "rebot_pinocchio_regressor.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr double kStructuralZeroTolerance = 1e-14;
constexpr double kNearZeroColumnRelativeTolerance = 1e-10;
constexpr std::array<double, 3> kRankThresholds{1e-5, 1e-6, 1e-7};
constexpr std::array<double, 4> kFrictionThresholds{0.005, 0.01, 0.02, 0.05};

struct BaseSpace {
  Eigen::VectorXd scales;
  Eigen::MatrixXd directions;
  Eigen::VectorXd singular_values;
  std::size_t rank{0};
  double effective_condition{std::numeric_limits<double>::infinity()};
};

struct RankDiagnostic {
  Eigen::Index raw_columns{0};
  Eigen::VectorXd column_norms;
  Eigen::VectorXd singular_values;
  std::vector<Eigen::Index> structural_zero_columns;
  std::vector<Eigen::Index> near_zero_columns;
};

std::string argumentValue(int argc, char **argv, const std::string &flag) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == flag) return argv[i + 1];
  }
  return "";
}

double requiredDouble(int argc, char **argv, const std::string &flag) {
  const std::string value = argumentValue(argc, argv, flag);
  if (value.empty()) throw std::runtime_error("missing " + flag);
  const double parsed = std::stod(value);
  if (!std::isfinite(parsed)) throw std::runtime_error("nonfinite " + flag);
  return parsed;
}

fs::path requiredPath(int argc, char **argv, const std::string &flag) {
  const std::string value = argumentValue(argc, argv, flag);
  if (value.empty()) throw std::runtime_error("missing " + flag);
  return fs::path(value);
}

Eigen::VectorXd flattenTorque(const ExperimentData &data) {
  Eigen::VectorXd result(static_cast<Eigen::Index>(data.n_samples * data.n_dof));
  for (std::size_t sample = 0; sample < data.n_samples; ++sample) {
    for (std::size_t joint = 0; joint < data.n_dof; ++joint) {
      result(static_cast<Eigen::Index>(sample * data.n_dof + joint)) =
          data.tau(static_cast<Eigen::Index>(sample), static_cast<Eigen::Index>(joint));
    }
  }
  return result;
}

std::vector<Eigen::Index> movingRows(const ExperimentData &data, double threshold,
                                     std::vector<std::size_t> *per_joint = nullptr) {
  std::vector<Eigen::Index> rows;
  if (per_joint) per_joint->assign(data.n_dof, 0);
  for (std::size_t sample = 0; sample < data.n_samples; ++sample) {
    for (std::size_t joint = 0; joint < data.n_dof; ++joint) {
      if (std::abs(data.qd(static_cast<Eigen::Index>(sample),
                           static_cast<Eigen::Index>(joint))) >= threshold) {
        rows.push_back(static_cast<Eigen::Index>(sample * data.n_dof + joint));
        if (per_joint) ++(*per_joint)[joint];
      }
    }
  }
  return rows;
}

Eigen::MatrixXd takeRows(const Eigen::MatrixXd &matrix,
                         const std::vector<Eigen::Index> &rows) {
  Eigen::MatrixXd result(static_cast<Eigen::Index>(rows.size()), matrix.cols());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    result.row(static_cast<Eigen::Index>(i)) = matrix.row(rows[i]);
  }
  return result;
}

Eigen::VectorXd takeRows(const Eigen::VectorXd &vector,
                         const std::vector<Eigen::Index> &rows) {
  Eigen::VectorXd result(static_cast<Eigen::Index>(rows.size()));
  for (std::size_t i = 0; i < rows.size(); ++i) {
    result(static_cast<Eigen::Index>(i)) = vector(rows[i]);
  }
  return result;
}

RankDiagnostic analyzeColumns(const Eigen::MatrixXd &observation) {
  RankDiagnostic result;
  result.raw_columns = observation.cols();
  result.column_norms.resize(observation.cols());
  double maximum = 0.0;
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    const double norm = observation.col(column).norm();
    result.column_norms(column) = norm;
    maximum = std::max(maximum, norm);
  }
  const double near_zero = kNearZeroColumnRelativeTolerance * maximum;
  Eigen::MatrixXd scaled = observation;
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    const double norm = result.column_norms(column);
    if (norm <= kStructuralZeroTolerance) {
      result.structural_zero_columns.push_back(column);
      scaled.col(column).setZero();
    } else {
      if (norm <= near_zero) result.near_zero_columns.push_back(column);
      scaled.col(column) /= norm;
    }
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled, Eigen::ComputeThinV);
  result.singular_values = svd.singularValues();
  return result;
}

std::size_t numericalRank(const Eigen::VectorXd &singular_values, double tolerance) {
  if (singular_values.size() == 0) return 0;
  const double threshold = tolerance * singular_values(0);
  return static_cast<std::size_t>(
      (singular_values.array() > threshold).count());
}

double effectiveCondition(const Eigen::VectorXd &singular_values, double tolerance) {
  const std::size_t rank = numericalRank(singular_values, tolerance);
  if (rank == 0) return std::numeric_limits<double>::infinity();
  return singular_values(0) /
         singular_values(static_cast<Eigen::Index>(rank - 1));
}

BaseSpace computeBaseSpace(const Eigen::MatrixXd &observation, double tolerance) {
  BaseSpace space;
  space.scales.resize(observation.cols());
  Eigen::MatrixXd scaled = observation;
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    double scale = observation.col(column).norm();
    if (!(scale > std::numeric_limits<double>::epsilon())) scale = 1.0;
    space.scales(column) = scale;
    scaled.col(column) /= scale;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled, Eigen::ComputeThinV);
  space.singular_values = svd.singularValues();
  space.rank = numericalRank(space.singular_values, tolerance);
  if (space.rank == 0) throw std::runtime_error("A regressor numerical rank is zero");
  space.directions = svd.matrixV().leftCols(static_cast<Eigen::Index>(space.rank));
  space.effective_condition =
      space.singular_values(0) /
      space.singular_values(static_cast<Eigen::Index>(space.rank - 1));
  return space;
}

Eigen::MatrixXd baseObservation(const Eigen::MatrixXd &observation,
                                const BaseSpace &space) {
  Eigen::MatrixXd scaled = observation;
  for (Eigen::Index column = 0; column < observation.cols(); ++column)
    scaled.col(column) /= space.scales(column);
  return scaled * space.directions;
}

void writeIndexList(std::ofstream &out, const std::string &key,
                    const std::vector<Eigen::Index> &values) {
  out << "    " << key << ": [";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) out << ", ";
    out << values[i];
  }
  out << "]\n";
}

void writeRank(std::ofstream &out, const std::string &name,
               const RankDiagnostic &diagnostic) {
  out << "  " << name << ":\n";
  out << "    raw_columns: " << diagnostic.raw_columns << "\n";
  writeIndexList(out, "structural_zero_columns", diagnostic.structural_zero_columns);
  writeIndexList(out, "near_zero_columns", diagnostic.near_zero_columns);
  out << "    thresholds:\n";
  for (double tolerance : kRankThresholds) {
    const std::size_t rank = numericalRank(diagnostic.singular_values, tolerance);
    out << "      - relative_tolerance: " << tolerance << "\n";
    out << "        rank: " << rank << "\n";
    out << "        sigma_max: " << diagnostic.singular_values(0) << "\n";
    out << "        smallest_retained_sigma: "
        << (rank ? diagnostic.singular_values(static_cast<Eigen::Index>(rank - 1)) : 0.0)
        << "\n";
    out << "        effective_condition: "
        << effectiveCondition(diagnostic.singular_values, tolerance) << "\n";
    out << "        near_zero_singular_directions: "
        << (diagnostic.raw_columns - static_cast<Eigen::Index>(rank)) << "\n";
  }
  out << "    singular_values: [";
  for (Eigen::Index i = 0; i < diagnostic.singular_values.size(); ++i) {
    if (i) out << ", ";
    out << diagnostic.singular_values(i);
  }
  out << "]\n";
}

}  // namespace

int main(int argc, char **argv) {
  try {
    std::cout << "OFFLINE_ONLY\n";
    const fs::path input = requiredPath(argc, argv, "--input");
    const fs::path model = requiredPath(argc, argv, "--model");
    const fs::path output = requiredPath(argc, argv, "--output");
    const fs::path prediction_output = requiredPath(argc, argv, "--prediction-output");
    const double fit_tolerance = requiredDouble(argc, argv, "--rank-relative-tolerance");
    const double friction_threshold =
        requiredDouble(argc, argv, "--friction-velocity-threshold");
    if (!(fit_tolerance > 0.0 && fit_tolerance < 1.0))
      throw std::runtime_error("rank tolerance must be in (0,1)");
    if (!(friction_threshold > 0.0))
      throw std::runtime_error("friction threshold must be positive");
    if (fs::exists(output) || fs::exists(prediction_output))
      throw std::runtime_error("outputs already exist; use a new audit directory");

    DataColumnSelection columns;
    columns.time_column = "time";
    columns.position_prefix = "q";
    columns.velocity_prefix = "qd";
    columns.acceleration_prefix = "qdd_est";
    columns.torque_prefix = "effort_filtered";
    columns.require_quality_columns = false;
    const ExperimentData data = DataLoader::loadCSV(input.string(), 6, columns);
    if (data.n_samples < 2 || !data.q.allFinite() || !data.qd.allFinite() ||
        !data.qdd.allFinite() || !data.tau.allFinite())
      throw std::runtime_error("real processed A requires finite six-axis samples");
    for (std::size_t i = 0; i < data.n_samples; ++i)
      if (!std::isfinite(data.time[i]) || (i && data.time[i] <= data.time[i - 1]))
        throw std::runtime_error("processed A time must be strictly increasing");

    rebot_dynamics::ReBotPinocchioRegressor regressor(model);
    using mujoco_dynamics::MuJoCoParamFlags;
    const Eigen::MatrixXd w60 = regressor.computeObservationMatrix(
        data.q.transpose(), data.qd.transpose(), data.qdd.transpose(),
        MuJoCoParamFlags::NONE);
    const Eigen::MatrixXd w72 = regressor.computeObservationMatrix(
        data.q.transpose(), data.qd.transpose(), data.qdd.transpose(),
        MuJoCoParamFlags::ALL);
    const Eigen::MatrixXd w78 = regressor.computeObservationMatrix(
        data.q.transpose(), data.qd.transpose(), data.qdd.transpose(),
        MuJoCoParamFlags::ALL_WITH_FRICTION);

    const RankDiagnostic rank60 = analyzeColumns(w60);
    const RankDiagnostic rank72 = analyzeColumns(w72);
    const RankDiagnostic rank78 = analyzeColumns(w78);

    std::vector<std::size_t> fit_counts;
    const auto selected = movingRows(data, friction_threshold, &fit_counts);
    if (std::any_of(fit_counts.begin(), fit_counts.end(),
                    [](std::size_t count) { return count < 2; }))
      throw std::runtime_error("each joint needs at least two moving observations");
    const BaseSpace space = computeBaseSpace(takeRows(w78, selected), fit_tolerance);
    if (selected.size() <= space.rank)
      throw std::runtime_error("insufficient selected observations for fitted rank");

    const Eigen::VectorXd target = flattenTorque(data);
    auto solver = identification::createAlgorithm("OLS", 6);
    const Eigen::MatrixXd selected_base =
        baseObservation(takeRows(w78, selected), space);
    const Eigen::VectorXd beta =
        solver->solve(selected_base, takeRows(target, selected));
    const Eigen::VectorXd prediction = baseObservation(w78, space) * beta;
    if (!beta.allFinite() || !prediction.allFinite())
      throw std::runtime_error("nonfinite A-only reported-effort fit");

    if (output.has_parent_path()) fs::create_directories(output.parent_path());
    if (prediction_output.has_parent_path())
      fs::create_directories(prediction_output.parent_path());

    std::ofstream out(output);
    std::ofstream predictions(prediction_output);
    if (!out || !predictions) throw std::runtime_error("cannot create diagnostic outputs");
    out << std::setprecision(17);
    out << "schema_version: rebot_real_a_training_diagnostic_v1\n";
    out << "data_mode: real_reported_effort\n";
    out << "acceleration_source: qdd_est\n";
    out << "torque_source: effort_filtered\n";
    out << "torque_calibrated: false\n";
    out << "physical_parameter_accuracy_claimed: false\n";
    out << "training_diagnostic_only: true\n";
    out << "input_file: " << std::quoted(input.string()) << "\n";
    out << "model_file: " << std::quoted(model.string()) << "\n";
    out << "sample_count: " << data.n_samples << "\n";
    out << "rank_sweep:\n";
    writeRank(out, "rank_60", rank60);
    writeRank(out, "rank_72", rank72);
    writeRank(out, "rank_78", rank78);
    out << "friction_moving_observations:\n";
    for (double threshold : kFrictionThresholds) {
      std::vector<std::size_t> counts;
      const auto rows = movingRows(data, threshold, &counts);
      out << "  - threshold_rad_s: " << threshold << "\n";
      out << "    total_count: " << rows.size() << "\n";
      out << "    per_joint_count: [";
      for (std::size_t j = 0; j < counts.size(); ++j) {
        if (j) out << ", ";
        out << counts[j];
      }
      out << "]\n";
      out << "    per_joint_fraction: [";
      for (std::size_t j = 0; j < counts.size(); ++j) {
        if (j) out << ", ";
        out << static_cast<double>(counts[j]) / static_cast<double>(data.n_samples);
      }
      out << "]\n";
    }
    out << "training_fit:\n";
    out << "  parameter_layout_columns: 78\n";
    out << "  rank_relative_tolerance: " << fit_tolerance << "\n";
    out << "  base_rank: " << space.rank << "\n";
    out << "  effective_condition: " << space.effective_condition << "\n";
    out << "  friction_velocity_threshold_rad_s: " << friction_threshold << "\n";
    out << "  included_observation_count: " << selected.size() << "\n";
    out << "  per_joint_included_count: [";
    for (std::size_t j = 0; j < fit_counts.size(); ++j) {
      if (j) out << ", ";
      out << fit_counts[j];
    }
    out << "]\n";
    out << "  prediction_file: " << std::quoted(prediction_output.string()) << "\n";

    predictions << std::setprecision(17)
                << "time,joint,effort_filtered,effort_predicted,residual,included\n";
    std::vector<bool> included(data.n_samples * data.n_dof, false);
    for (Eigen::Index row : selected) included[static_cast<std::size_t>(row)] = true;
    for (std::size_t sample = 0; sample < data.n_samples; ++sample) {
      for (std::size_t joint = 0; joint < data.n_dof; ++joint) {
        const std::size_t index = sample * data.n_dof + joint;
        predictions << data.time[sample] << ',' << joint + 1 << ','
                    << target(static_cast<Eigen::Index>(index)) << ','
                    << prediction(static_cast<Eigen::Index>(index)) << ','
                    << prediction(static_cast<Eigen::Index>(index)) -
                           target(static_cast<Eigen::Index>(index))
                    << ',' << (included[index] ? 1 : 0) << '\n';
      }
    }

    std::cout << "data_mode=real_reported_effort\n"
              << "acceleration_source=qdd_est\n"
              << "torque_source=effort_filtered\n"
              << "torque_calibrated=false\n"
              << "training_diagnostic_only=true\n"
              << "samples=" << data.n_samples
              << " fit_rank=" << space.rank
              << " fit_condition=" << space.effective_condition << "\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "A-only offline diagnostic failed: " << error.what() << "\n";
    return 1;
  }
}
