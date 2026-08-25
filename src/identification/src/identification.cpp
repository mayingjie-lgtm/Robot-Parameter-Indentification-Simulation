#include "identification/identification.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>

using namespace mujoco_dynamics;

namespace {

std::string normalizeRobotType(std::string robot_type) {
  std::transform(robot_type.begin(), robot_type.end(), robot_type.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return robot_type;
}

MuJoCoParamFlags effectiveFlagsForAlgorithm(const std::string &algorithm_type,
                                            MuJoCoParamFlags flags) {
  if (algorithm_type == "NLS_FRICTION") {
    return hasFlag(flags, MuJoCoParamFlags::ARMATURE)
               ? MuJoCoParamFlags::ARMATURE
               : MuJoCoParamFlags::NONE;
  }
  return flags;
}

} // namespace

Identification::Identification(const std::string &robot_type,
                               const std::filesystem::path &model_path,
                               const std::vector<double> &frictionloss,
                               const std::vector<double> &armature,
                               const std::vector<double> &damping,
                               std::unique_ptr<robot::RobotModel> model)
    : model_(std::move(model)), robot_type_(normalizeRobotType(robot_type)),
      piper_regressor_(
          robot_type_ == "piper" && !model_path.empty()
              ? model_path
              : std::filesystem::path(PROJECT_ROOT_DIR) / "piper" / "piper.xml",
          {0.035, -0.035}, robot_type_ == "piper" ? frictionloss
                                                   : std::vector<double>{}) {
  if (robot_type_ == "rebot_dm") {
    const std::filesystem::path rebot_urdf =
        model_path.empty()
            ? std::filesystem::path(PROJECT_ROOT_DIR) / "rebot_dm" /
                  "rebot_dm.urdf"
            : model_path;
    rebot_regressor_ = std::make_unique<rebot_dynamics::ReBotPinocchioRegressor>(
        rebot_urdf, armature, damping, frictionloss);
  }
}

void Identification::preprocess(ExperimentData &data) {
  if (data.qdd.rows() == static_cast<Eigen::Index>(data.n_samples) &&
      data.qdd.cols() == static_cast<Eigen::Index>(data.n_dof)) {
    std::cout << "Preprocessing: using qdd from physics engine (already loaded)"
              << std::endl;
    std::cout << "DEBUG: qdd max abs: " << data.qdd.cwiseAbs().maxCoeff()
              << std::endl;
    std::cout << "DEBUG: qdd norm: " << data.qdd.norm() << std::endl;
    return;
  }

  std::cout << "Preprocessing: computing qdd via numerical differentiation..."
            << std::endl;

  data.qdd.resize(data.n_samples, data.n_dof);
  for (std::size_t i = 1; i < data.n_samples - 1; ++i) {
    const double dt = data.time[i + 1] - data.time[i - 1];
    if (dt < 1e-6) {
      continue;
    }
    for (std::size_t j = 0; j < data.n_dof; ++j) {
      data.qdd(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
          (data.qd(static_cast<Eigen::Index>(i + 1), static_cast<Eigen::Index>(j)) -
           data.qd(static_cast<Eigen::Index>(i - 1), static_cast<Eigen::Index>(j))) /
          dt;
    }
  }

  data.qdd.row(0) = data.qdd.row(1);
  data.qdd.row(static_cast<Eigen::Index>(data.n_samples - 1)) =
      data.qdd.row(static_cast<Eigen::Index>(data.n_samples - 2));

  std::cout << "Preprocessing complete: computed accelerations." << std::endl;
  std::cout << "DEBUG: qdd max abs: " << data.qdd.cwiseAbs().maxCoeff()
            << std::endl;
  std::cout << "DEBUG: qdd norm: " << data.qdd.norm() << std::endl;
}

std::size_t Identification::numParameters(MuJoCoParamFlags flags) const {
  if (robot_type_ == "piper") {
    return piper_regressor_.numParameters(flags);
  }
  if (robot_type_ == "rebot_dm") {
    return rebot_regressor_->numParameters(flags);
  }
  return panda_regressor_.numParameters(flags);
}

Eigen::VectorXd
Identification::getGroundTruthParameters(MuJoCoParamFlags flags) const {
  if (robot_type_ == "piper") {
    return piper_regressor_.computeParameterVector(flags);
  }
  if (robot_type_ == "rebot_dm") {
    return rebot_regressor_->computeParameterVector(flags);
  }
  return panda_regressor_.computeParameterVector(flags);
}

Eigen::MatrixXd Identification::computeObservationMatrix(
    const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qd,
    const Eigen::MatrixXd &Qdd, MuJoCoParamFlags flags) const {
  if (robot_type_ == "piper") {
    return piper_regressor_.computeObservationMatrix(Q, Qd, Qdd, flags);
  }
  if (robot_type_ == "rebot_dm") {
    return rebot_regressor_->computeObservationMatrix(Q, Qd, Qdd, flags);
  }
  return panda_regressor_.computeObservationMatrix(Q, Qd, Qdd, flags);
}

Eigen::VectorXd Identification::solve(const ExperimentData &data,
                                      const std::string &algorithm_type,
                                      MuJoCoParamFlags flags) {
  if (data.qdd.rows() == 0) {
    throw std::runtime_error("Experiment data not preprocessed: qdd is empty");
  }

  const char *regressor_name =
      robot_type_ == "piper"
          ? "MuJoCoPiperRegressor"
          : (robot_type_ == "rebot_dm" ? "ReBotPinocchioRegressor"
                                         : "MuJoCoRegressor");
  std::cout << "Building observation matrix W (using " << regressor_name
            << ")..." << std::endl;

  if (!data.q.allFinite() || !data.qd.allFinite() || !data.qdd.allFinite() ||
      !data.tau.allFinite()) {
    throw std::runtime_error(
        "Identification::solve received non-finite data; filter by explicit "
        "quality fields before solving");
  }

  Eigen::VectorXd tau_meas(data.n_samples * data.n_dof);
  for (std::size_t sample = 0; sample < data.n_samples; ++sample) {
    for (std::size_t joint = 0; joint < data.n_dof; ++joint) {
      tau_meas(static_cast<Eigen::Index>(sample * data.n_dof + joint)) =
          data.tau(static_cast<Eigen::Index>(sample),
                   static_cast<Eigen::Index>(joint));
    }
  }

  const Eigen::MatrixXd W = computeObservationMatrix(
      data.q.transpose(), data.qd.transpose(), data.qdd.transpose(),
      effectiveFlagsForAlgorithm(algorithm_type, flags));

  std::cout << "W size: " << W.rows() << " x " << W.cols() << std::endl;

  auto solver = identification::createAlgorithm(algorithm_type, data.n_dof);
  if (!solver) {
    std::cout << "WARNING: " << algorithm_type
              << " algorithm is not fully supported with current regressor. "
              << "Falling back to OLS." << std::endl;
    solver = identification::createAlgorithm("OLS", data.n_dof);
  }

  if (auto *nonlinear_solver =
          dynamic_cast<identification::NonlinearFrictionLM *>(solver.get())) {
    nonlinear_solver->setVelocityData(data.qd);
  }

  std::cout << "Solving using " << algorithm_type << "..." << std::endl;
  return solver->solve(W, tau_meas);
}

Eigen::VectorXd Identification::predictTorques(const ExperimentData &data,
                                               const Eigen::VectorXd &params,
                                               const std::string &algorithm_type,
                                               MuJoCoParamFlags flags) const {
  if (data.qdd.rows() == 0) {
    throw std::runtime_error("Experiment data not preprocessed: qdd is empty");
  }

  const Eigen::MatrixXd W = computeObservationMatrix(
      data.q.transpose(), data.qd.transpose(), data.qdd.transpose(),
      effectiveFlagsForAlgorithm(algorithm_type, flags));

  if (algorithm_type == "NLS_FRICTION") {
    return identification::NonlinearFrictionLM::predictTorques(
        W, data.qd, params, static_cast<int>(data.n_dof));
  }

  return W * params;
}
