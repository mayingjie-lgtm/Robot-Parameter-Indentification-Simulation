#include "identification/data_loader.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace {

/** Split a comma-separated row without guessing its semantic layout. */
std::vector<std::string> splitCSV(const std::string &line) {
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) {
    values.push_back(value);
  }
  return values;
}

/** Resolve one required header and include its physical name in errors. */
std::size_t requiredColumn(
    const std::unordered_map<std::string, std::size_t> &columns,
    const std::string &name) {
  const auto found = columns.find(name);
  if (found == columns.end()) {
    throw std::runtime_error("CSV 缺少明确要求的列: " + name);
  }
  return found->second;
}

/** Resolve a complete numbered joint-column group. */
std::vector<std::size_t> requiredJointColumns(
    const std::unordered_map<std::string, std::size_t> &columns,
    const std::string &prefix, std::size_t n_dof) {
  std::vector<std::size_t> result;
  result.reserve(n_dof);
  for (std::size_t joint = 0; joint < n_dof; ++joint) {
    result.push_back(requiredColumn(columns, prefix + std::to_string(joint)));
  }
  return result;
}

/** Convert one CSV token and report the source row on malformed input. */
double parseValue(const std::vector<std::string> &row, std::size_t column,
                  std::size_t line_number) {
  if (column >= row.size()) {
    throw std::runtime_error("CSV 第 " + std::to_string(line_number) +
                             " 行列数不足");
  }
  try {
    return std::stod(row[column]);
  } catch (const std::exception &) {
    throw std::runtime_error("CSV 第 " + std::to_string(line_number) +
                             " 行包含无法解析的数值");
  }
}

} // namespace

ExperimentData DataLoader::loadCSV(const std::string &filename,
                                   std::size_t n_dof,
                                   const DataColumnSelection &selection) {
  std::ifstream file(filename);
  if (!file) {
    throw std::runtime_error("Could not open file: " + filename);
  }

  std::string line;
  if (!std::getline(file, line)) {
    throw std::runtime_error("CSV 为空: " + filename);
  }
  const auto header = splitCSV(line);
  std::unordered_map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < header.size(); ++i) {
    if (!columns.emplace(header[i], i).second) {
      throw std::runtime_error("CSV header 包含重复列: " + header[i]);
    }
  }

  const std::size_t time_column =
      requiredColumn(columns, selection.time_column);
  const auto q_columns = requiredJointColumns(
      columns, selection.position_prefix, n_dof);
  const auto qd_columns = requiredJointColumns(
      columns, selection.velocity_prefix, n_dof);
  const auto qdd_columns = requiredJointColumns(
      columns, selection.acceleration_prefix, n_dof);
  const auto tau_columns = requiredJointColumns(
      columns, selection.torque_prefix, n_dof);

  std::vector<std::size_t> constraint_columns;
  std::size_t saturated_column = 0;
  std::size_t contact_column = 0;
  if (selection.require_quality_columns) {
    constraint_columns =
        requiredJointColumns(columns, "tau_constraint", n_dof);
    saturated_column = requiredColumn(columns, "saturated");
    contact_column = requiredColumn(columns, "contact_count");
  }

  std::vector<double> time;
  std::vector<std::vector<double>> q_rows;
  std::vector<std::vector<double>> qd_rows;
  std::vector<std::vector<double>> qdd_rows;
  std::vector<std::vector<double>> tau_rows;
  std::vector<std::vector<double>> constraint_rows;
  std::vector<int> saturated;
  std::vector<int> contacts;
  std::size_t line_number = 1;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }
    const auto row = splitCSV(line);
    std::vector<double> q_row(n_dof);
    std::vector<double> qd_row(n_dof);
    std::vector<double> qdd_row(n_dof);
    std::vector<double> tau_row(n_dof);
    std::vector<double> constraint_row(n_dof, 0.0);
    for (std::size_t joint = 0; joint < n_dof; ++joint) {
      q_row[joint] = parseValue(row, q_columns[joint], line_number);
      qd_row[joint] = parseValue(row, qd_columns[joint], line_number);
      qdd_row[joint] = parseValue(row, qdd_columns[joint], line_number);
      tau_row[joint] = parseValue(row, tau_columns[joint], line_number);
      if (selection.require_quality_columns) {
        constraint_row[joint] =
            parseValue(row, constraint_columns[joint], line_number);
      }
    }
    time.push_back(parseValue(row, time_column, line_number));
    q_rows.push_back(std::move(q_row));
    qd_rows.push_back(std::move(qd_row));
    qdd_rows.push_back(std::move(qdd_row));
    tau_rows.push_back(std::move(tau_row));
    constraint_rows.push_back(std::move(constraint_row));
    saturated.push_back(selection.require_quality_columns
                            ? static_cast<int>(parseValue(
                                  row, saturated_column, line_number))
                            : 0);
    contacts.push_back(selection.require_quality_columns
                           ? static_cast<int>(parseValue(
                                 row, contact_column, line_number))
                           : 0);
  }

  ExperimentData data;
  data.n_samples = time.size();
  data.n_dof = n_dof;
  data.time = std::move(time);
  data.q.resize(data.n_samples, n_dof);
  data.qd.resize(data.n_samples, n_dof);
  data.qdd.resize(data.n_samples, n_dof);
  data.tau.resize(data.n_samples, n_dof);
  data.tau_constraint.resize(data.n_samples, n_dof);
  data.saturated = std::move(saturated);
  data.contact_count = std::move(contacts);
  for (std::size_t sample = 0; sample < data.n_samples; ++sample) {
    for (std::size_t joint = 0; joint < n_dof; ++joint) {
      const auto row = static_cast<Eigen::Index>(sample);
      const auto col = static_cast<Eigen::Index>(joint);
      data.q(row, col) = q_rows[sample][joint];
      data.qd(row, col) = qd_rows[sample][joint];
      data.qdd(row, col) = qdd_rows[sample][joint];
      data.tau(row, col) = tau_rows[sample][joint];
      data.tau_constraint(row, col) = constraint_rows[sample][joint];
    }
  }

  std::cout << "Loaded " << data.n_samples << " samples from " << filename
            << " using exact columns " << selection.position_prefix << "/"
            << selection.velocity_prefix << "/"
            << selection.acceleration_prefix << "/" << selection.torque_prefix
            << std::endl;
  return data;
}
