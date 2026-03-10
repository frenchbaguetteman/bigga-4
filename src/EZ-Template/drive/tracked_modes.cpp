/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"

#include "config.h"
#include "motionProfiling/motionProfile.h"
#include "motionProfiling/path.h"
#include "motionProfiling/profileBuilder.h"
#include "velocityProfile/trapezoidalVelocityProfile.hpp"

#include "Eigen/Dense"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace ez;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInToM = 0.0254;
constexpr double kMToIn = 1.0 / kInToM;
constexpr double kTrackingPathEpsilonM = 1e-4;
constexpr double kAngularEpsilon = 1e-5;
constexpr double kCostEpsilon = 1e-4;
constexpr int kDefaultTrackingSamples = 200;

struct TrackingSampleData {
  pose pose_target{0.0, 0.0, 0.0};
  double x_m = 0.0;
  double y_m = 0.0;
  double theta_rad = 0.0;
  double linear_velocity_mps = 0.0;
  double angular_velocity_rps = 0.0;
  double linear_accel_mps2 = 0.0;
  double time_sec = 0.0;
};

struct TrackingGainData {
  double k00 = 0.0;
  double k01 = 0.0;
  double k02 = 0.0;
  double k10 = 0.0;
  double k11 = 0.0;
  double k12 = 0.0;
};

struct BuiltTrackingPath {
  std::vector<TrackingSampleData> samples;
  std::vector<int> waypointSampleIndex;
  std::vector<pose> waypointTargets;
  pose goal{0.0, 0.0, ANGLE_NOT_SET};
  double totalTimeSec = 0.0;
  bool valid = false;
};

struct InternalPose {
  double x_m = 0.0;
  double y_m = 0.0;
  double theta_rad = 0.0;
};

using StateVector = Eigen::Vector3d;
using ControlVector = Eigen::Vector2d;
using StateMatrix = Eigen::Matrix3d;
using ControlMatrix = Eigen::Matrix2d;
using InputMatrix = Eigen::Matrix<double, 3, 2>;
using GainMatrix = Eigen::Matrix<double, 2, 3>;

double clampAbs(double input, double maxAbs) {
  const double limited = std::fabs(maxAbs);
  return std::clamp(input, -limited, limited);
}

double wrapRadians(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double angleDiffRad(double target, double current) {
  return wrapRadians(target - current);
}

double sinc(double angle) {
  if (std::fabs(angle) < 1e-6) {
    return 1.0 - (angle * angle) / 6.0;
  }
  return std::sin(angle) / angle;
}

InternalPose toInternalPose(const pose& ezPose) {
  return {
      ezPose.x * kInToM,
      ezPose.y * kInToM,
      wrapRadians((kPi / 2.0) - ez::util::to_rad(ezPose.theta)),
  };
}

pose toEzPose(double x_m, double y_m, double theta_rad) {
  return {
      x_m * kMToIn,
      y_m * kMToIn,
      ez::util::wrap_angle(90.0 - ez::util::to_deg(theta_rad)),
  };
}

double pointDistanceM(const pose& a, const pose& b) {
  const double dx = (b.x - a.x) * kInToM;
  const double dy = (b.y - a.y) * kInToM;
  return std::sqrt(dx * dx + dy * dy);
}

double tangentHeadingRad(const std::vector<pose>& points, std::size_t index, double fallbackHeading) {
  if (points.empty()) {
    return fallbackHeading;
  }

  auto chooseHeading = [&](std::size_t from, std::size_t to) -> double {
    const InternalPose fromPose = toInternalPose(points[from]);
    const InternalPose toPose = toInternalPose(points[to]);
    const double dx = toPose.x_m - fromPose.x_m;
    const double dy = toPose.y_m - fromPose.y_m;
    if (std::hypot(dx, dy) <= kTrackingPathEpsilonM) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return std::atan2(dy, dx);
  };

  if (index + 1 < points.size()) {
    const double heading = chooseHeading(index, index + 1);
    if (std::isfinite(heading)) {
      return heading;
    }
  }

  if (index > 0) {
    const double heading = chooseHeading(index - 1, index);
    if (std::isfinite(heading)) {
      return heading;
    }
  }

  return fallbackHeading;
}

BuiltTrackingPath buildTrackedPath(const std::vector<pose>& ezPoints,
                                   const pose& explicitFinalPose,
                                   double maxVelocityMps,
                                   double maxAccelerationMps2) {
  BuiltTrackingPath built;

  if (ezPoints.size() < 2) {
    return built;
  }

  std::vector<InternalPose> internalPoints;
  internalPoints.reserve(ezPoints.size());
  for (const pose& point : ezPoints) {
    internalPoints.push_back(toInternalPose(point));
  }

  double totalLength = 0.0;
  for (std::size_t i = 1; i < internalPoints.size(); ++i) {
    totalLength += std::hypot(internalPoints[i].x_m - internalPoints[i - 1].x_m,
                              internalPoints[i].y_m - internalPoints[i - 1].y_m);
  }
  if (totalLength <= kTrackingPathEpsilonM) {
    return built;
  }

  std::vector<Waypoint> pathWaypoints;
  pathWaypoints.reserve(ezPoints.size());

  const double startHeading = toInternalPose(ezPoints.front()).theta_rad;
  for (std::size_t i = 0; i < ezPoints.size(); ++i) {
    const InternalPose internal = internalPoints[i];
    double heading = tangentHeadingRad(ezPoints, i, startHeading);
    if (i + 1 == ezPoints.size() && explicitFinalPose.theta != ANGLE_NOT_SET) {
      heading = toInternalPose(explicitFinalPose).theta_rad;
    }
    pathWaypoints.push_back({static_cast<float>(internal.x_m),
                             static_cast<float>(internal.y_m),
                             static_cast<float>(heading)});
  }

  const MotionProfile profile = buildProfile(
      Path(pathWaypoints),
      ProfileConstraints{
          QSpeed(std::fabs(maxVelocityMps)),
          QAcceleration(std::fabs(maxAccelerationMps2)),
      },
      kDefaultTrackingSamples,
      QSpeed(0.0),
      QSpeed(0.0));

  const auto& profileSamples = profile.samples();
  if (profileSamples.empty()) {
    return built;
  }

  const double totalTimeSec = std::max(0.0f, profile.totalTime());
  const double dt = profileSamples.size() > 1
      ? totalTimeSec / static_cast<double>(profileSamples.size() - 1)
      : 0.0;

  built.samples.reserve(profileSamples.size());
  for (std::size_t i = 0; i < profileSamples.size(); ++i) {
    const ProfileState& sample = profileSamples[i];
    const pose ezPose = toEzPose(sample.pose.x(), sample.pose.y(), sample.pose.z());
    built.samples.push_back({
        ezPose,
        sample.pose.x(),
        sample.pose.y(),
        sample.pose.z(),
        sample.linearVelocity,
        sample.angularVelocity,
        sample.linearAcceleration,
        dt * static_cast<double>(i),
    });
  }

  built.goal = built.samples.back().pose_target;
  if (explicitFinalPose.theta != ANGLE_NOT_SET) {
    built.goal.theta = explicitFinalPose.theta;
  }

  built.totalTimeSec = totalTimeSec;
  built.waypointSampleIndex.reserve(ezPoints.size() - 1);
  built.waypointTargets.reserve(ezPoints.size() - 1);
  int previousIndex = 0;
  for (std::size_t i = 1; i < ezPoints.size(); ++i) {
    const InternalPose waypoint = internalPoints[i];
    int bestIndex = previousIndex;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (int sampleIndex = previousIndex;
         sampleIndex < static_cast<int>(profileSamples.size());
         ++sampleIndex) {
      const ProfileState& sample = profileSamples[static_cast<std::size_t>(sampleIndex)];
      const double distance =
          std::hypot(sample.pose.x() - waypoint.x_m, sample.pose.y() - waypoint.y_m);
      if (distance <= bestDistance) {
        bestDistance = distance;
        bestIndex = sampleIndex;
      }
    }

    previousIndex = bestIndex;
    built.waypointSampleIndex.push_back(bestIndex);

    pose waypointTarget = ezPoints[i];
    if (i + 1 == ezPoints.size() && explicitFinalPose.theta != ANGLE_NOT_SET) {
      waypointTarget.theta = explicitFinalPose.theta;
    }
    built.waypointTargets.push_back(waypointTarget);
  }

  built.valid = true;
  return built;
}

StateMatrix sanitizeStateCost(const StateMatrix& cost) {
  StateMatrix sanitized = 0.5 * (cost + cost.transpose());
  for (int i = 0; i < 3; ++i) {
    sanitized(i, i) += kCostEpsilon;
  }
  return sanitized;
}

ControlMatrix sanitizeControlCost(const ControlMatrix& cost) {
  ControlMatrix sanitized = 0.5 * (cost + cost.transpose());
  for (int i = 0; i < 2; ++i) {
    sanitized(i, i) += kCostEpsilon;
  }
  return sanitized;
}

ControlMatrix inverse2x2(const ControlMatrix& matrix) {
  ControlMatrix inverse = ControlMatrix::Identity();
  double determinant = matrix(0, 0) * matrix(1, 1) - matrix(0, 1) * matrix(1, 0);
  if (std::fabs(determinant) < kCostEpsilon) {
    determinant = determinant < 0.0 ? -kCostEpsilon : kCostEpsilon;
  }

  const double invDet = 1.0 / determinant;
  inverse(0, 0) = matrix(1, 1) * invDet;
  inverse(0, 1) = -matrix(0, 1) * invDet;
  inverse(1, 0) = -matrix(1, 0) * invDet;
  inverse(1, 1) = matrix(0, 0) * invDet;
  return inverse;
}

struct DiscreteModel {
  StateMatrix A = StateMatrix::Identity();
  InputMatrix B = InputMatrix::Zero();
};

DiscreteModel discreteTrackingModel(double desiredVelocity,
                                    double desiredAngularVelocity,
                                    double dt) {
  DiscreteModel model;
  if (dt <= 0.0) {
    return model;
  }

  if (std::fabs(desiredAngularVelocity) < kAngularEpsilon) {
    model.A(1, 2) = desiredVelocity * dt;
    model.B(0, 0) = -dt;
    model.B(1, 1) = -0.5 * desiredVelocity * dt * dt;
    model.B(2, 1) = -dt;
    return model;
  }

  const double w = desiredAngularVelocity;
  const double wt = w * dt;
  const double cosWt = std::cos(wt);
  const double sinWt = std::sin(wt);
  const double invW = 1.0 / w;
  const double invW2 = invW * invW;

  model.A(0, 0) = cosWt;
  model.A(0, 1) = sinWt;
  model.A(0, 2) = desiredVelocity * (1.0 - cosWt) * invW;
  model.A(1, 0) = -sinWt;
  model.A(1, 1) = cosWt;
  model.A(1, 2) = desiredVelocity * sinWt * invW;

  model.B(0, 0) = -sinWt * invW;
  model.B(0, 1) = -desiredVelocity * (dt * invW - sinWt * invW2);
  model.B(1, 0) = (1.0 - cosWt) * invW;
  model.B(1, 1) = -desiredVelocity * ((1.0 - cosWt) * invW2);
  model.B(2, 1) = -dt;
  return model;
}

}  // namespace

void Drive::pid_ramsete_constants_set(double zeta, double beta) {
  ramsete_zeta = std::max(0.01, std::fabs(zeta));
  ramsete_beta = std::max(0.01, std::fabs(beta));
}

void Drive::pid_ltv_costs_set(double qx, double qy, double qtheta, double rv, double romega, double terminal_scale) {
  ltv_qx = std::max(std::fabs(qx), kCostEpsilon);
  ltv_qy = std::max(std::fabs(qy), kCostEpsilon);
  ltv_qtheta = std::max(std::fabs(qtheta), kCostEpsilon);
  ltv_rv = std::max(std::fabs(rv), kCostEpsilon);
  ltv_romega = std::max(std::fabs(romega), kCostEpsilon);
  ltv_terminal_scale = std::max(std::fabs(terminal_scale), 1.0);
}

void Drive::pid_odom_ramsete_set(odom imovement) {
  pid_odom_ramsete_set(imovement, slew_drive_forward_get());
}

void Drive::pid_odom_ramsete_set(odom imovement, bool slew_on) {
  pid_odom_ramsete_set(std::vector<odom>{imovement}, slew_on);
}

void Drive::pid_odom_ramsete_set(united_odom p_imovement) {
  pid_odom_ramsete_set(util::united_odom_to_odom(p_imovement));
}

void Drive::pid_odom_ramsete_set(united_odom p_imovement, bool slew_on) {
  pid_odom_ramsete_set(util::united_odom_to_odom(p_imovement), slew_on);
}

void Drive::pid_odom_ramsete_set(std::vector<odom> imovements) {
  const bool slew_on = imovements.empty() ? false : slew_drive_forward_get();
  pid_odom_ramsete_set(imovements, slew_on);
}

void Drive::pid_odom_ramsete_set(std::vector<odom> imovements, bool slew_on) {
  start_tracked_motion(std::move(imovements), slew_on, RAMSETE);
}

void Drive::pid_odom_ramsete_set(std::vector<united_odom> p_imovements) {
  pid_odom_ramsete_set(util::united_odoms_to_odoms(p_imovements));
}

void Drive::pid_odom_ramsete_set(std::vector<united_odom> p_imovements, bool slew_on) {
  pid_odom_ramsete_set(util::united_odoms_to_odoms(p_imovements), slew_on);
}

void Drive::pid_odom_ltv_set(odom imovement) {
  pid_odom_ltv_set(imovement, slew_drive_forward_get());
}

void Drive::pid_odom_ltv_set(odom imovement, bool slew_on) {
  pid_odom_ltv_set(std::vector<odom>{imovement}, slew_on);
}

void Drive::pid_odom_ltv_set(united_odom p_imovement) {
  pid_odom_ltv_set(util::united_odom_to_odom(p_imovement));
}

void Drive::pid_odom_ltv_set(united_odom p_imovement, bool slew_on) {
  pid_odom_ltv_set(util::united_odom_to_odom(p_imovement), slew_on);
}

void Drive::pid_odom_ltv_set(std::vector<odom> imovements) {
  const bool slew_on = imovements.empty() ? false : slew_drive_forward_get();
  pid_odom_ltv_set(imovements, slew_on);
}

void Drive::pid_odom_ltv_set(std::vector<odom> imovements, bool slew_on) {
  start_tracked_motion(std::move(imovements), slew_on, LTV);
}

void Drive::pid_odom_ltv_set(std::vector<united_odom> p_imovements) {
  pid_odom_ltv_set(util::united_odoms_to_odoms(p_imovements));
}

void Drive::pid_odom_ltv_set(std::vector<united_odom> p_imovements, bool slew_on) {
  pid_odom_ltv_set(util::united_odoms_to_odoms(p_imovements), slew_on);
}

bool Drive::tracked_mode_active() const {
  return tracked_controller != TRACKING_NONE && !tracked_samples.empty();
}

void Drive::reset_tracked_motion_state() {
  tracked_controller = TRACKING_NONE;
  tracked_samples.clear();
  tracked_ltv_gains.clear();
  tracked_waypoint_sample_index.clear();
  tracked_waypoints.clear();
  tracked_goal = {0.0, 0.0, ANGLE_NOT_SET};
  tracked_total_time_sec = 0.0;
  tracked_start_ms = 0;
  tracked_current_sample_index = 0;
  tracked_speed_cap = 0;
  tracked_last_left_cmd = 0.0;
  tracked_last_right_cmd = 0.0;
  tracked_slew_on = false;
}

void Drive::start_tracked_motion(std::vector<odom> imovements, bool slew_on, e_mode tracking_mode) {
  drive_mode_set(DISABLE, true);
  interfered = false;
  drive_toggle = true;

  if (imovements.empty()) {
    if (print_toggle) std::printf("Tracked motion rejected: empty waypoint list\n");
    return;
  }

  const drive_directions firstDirection = imovements.front().drive_direction;
  for (const odom& movement : imovements) {
    if (movement.drive_direction != firstDirection) {
      if (print_toggle) std::printf("Tracked motion rejected: mixed drive directions are unsupported\n");
      return;
    }
    if (movement.drive_direction == REV) {
      if (print_toggle) std::printf("Tracked motion rejected: reverse tracked paths are unsupported\n");
      return;
    }
  }

  std::vector<odom> input = set_odoms_direction(imovements);
  const pose startPose = odom_pose_get();
  input.insert(input.begin(), {{startPose.x, startPose.y, ANGLE_NOT_SET}, FWD, input.front().max_xy_speed});

  const int speedCap = std::clamp(std::min_element(
      imovements.begin(),
      imovements.end(),
      [](const odom& lhs, const odom& rhs) {
        return lhs.max_xy_speed < rhs.max_xy_speed;
      })->max_xy_speed,
      1,
      127);

  const double wheelLinearSpeedMps =
      ((CARTRIDGE / std::max(0.001, RATIO)) * (WHEEL_DIAMETER * M_PI * kInToM)) / 60.0;
  const double maxVelocityMps = std::min(
      static_cast<double>(CONFIG::MAX_SPEED.convert(mps)),
      wheelLinearSpeedMps * (static_cast<double>(speedCap) / 127.0));
  const double maxAccelerationMps2 = CONFIG::MAX_ACCELERATION.convert(mps2);

  std::vector<pose> ezPoints;
  ezPoints.reserve(input.size());
  for (const odom& movement : input) {
    pose point = movement.target;
    point.theta = ANGLE_NOT_SET;
    ezPoints.push_back(point);
  }

  const pose explicitFinalPose = input.back().target;
  const BuiltTrackingPath built = buildTrackedPath(
      ezPoints,
      explicitFinalPose,
      maxVelocityMps,
      maxAccelerationMps2);

  if (!built.valid) {
    if (print_toggle) std::printf("Tracked motion rejected: path length too short\n");
    return;
  }

  tracked_samples.reserve(built.samples.size());
  for (const TrackingSampleData& sample : built.samples) {
    tracked_samples.push_back({
        sample.pose_target,
        sample.x_m,
        sample.y_m,
        sample.theta_rad,
        sample.linear_velocity_mps,
        sample.angular_velocity_rps,
        sample.linear_accel_mps2,
        sample.time_sec,
    });
  }

  tracked_waypoint_sample_index = built.waypointSampleIndex;
  tracked_waypoints = built.waypointTargets;
  tracked_goal = built.goal;
  tracked_total_time_sec = built.totalTimeSec;
  tracked_speed_cap = speedCap;
  tracked_slew_on = slew_on;
  tracked_start_ms = pros::millis();
  tracked_current_sample_index = 0;
  current_drive_direction = FWD;
  point_to_face = find_point_to_face(startPose, tracked_goal, FWD, true);

  xyPID.timers_reset();
  current_a_odomPID.timers_reset();
  l_start = drive_sensor_left();
  r_start = drive_sensor_right();
  pid_speed_max_set(speedCap);

  if (tracking_mode == RAMSETE) {
    tracked_controller = TRACKING_RAMSETE;
  } else {
    tracked_controller = TRACKING_LTV;

    const double dt = tracked_samples.size() > 1
        ? tracked_total_time_sec / static_cast<double>(tracked_samples.size() - 1)
        : 0.0;
    StateMatrix q = StateMatrix::Zero();
    q(0, 0) = ltv_qx;
    q(1, 1) = ltv_qy;
    q(2, 2) = ltv_qtheta;
    q = sanitizeStateCost(q);

    ControlMatrix r = ControlMatrix::Zero();
    r(0, 0) = ltv_rv;
    r(1, 1) = ltv_romega;
    r = sanitizeControlCost(r);

    StateMatrix qf = StateMatrix::Zero();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        qf(row, col) = q(row, col) * ltv_terminal_scale;
      }
    }
    qf = sanitizeStateCost(qf);

    std::vector<StateMatrix> riccati(tracked_samples.size(), StateMatrix::Zero());
    riccati.back() = qf;
    tracked_ltv_gains.assign(tracked_samples.size(), tracked_gain{});

    for (int index = static_cast<int>(tracked_samples.size()) - 2; index >= 0; --index) {
      const tracked_sample& sample = tracked_samples[static_cast<std::size_t>(index)];
      const DiscreteModel model =
          discreteTrackingModel(sample.linear_velocity_mps,
                                sample.angular_velocity_rps,
                                dt);

      const StateMatrix& pNext = riccati[static_cast<std::size_t>(index + 1)];
      ControlMatrix solveMatrix = sanitizeControlCost(r + model.B.transpose() * pNext * model.B);
      const GainMatrix gain =
          inverse2x2(solveMatrix) * (model.B.transpose() * pNext * model.A);

      tracked_ltv_gains[static_cast<std::size_t>(index)] = {
          gain(0, 0), gain(0, 1), gain(0, 2),
          gain(1, 0), gain(1, 1), gain(1, 2),
      };

      StateMatrix current =
          q + model.A.transpose() * pNext * (model.A - model.B * gain);
      riccati[static_cast<std::size_t>(index)] = sanitizeStateCost(current);
    }

    if (tracked_ltv_gains.size() > 1) {
      tracked_ltv_gains.back() = tracked_ltv_gains[tracked_ltv_gains.size() - 2];
    }
  }

  if (print_toggle) {
    std::printf("%s Motion Started... %zu waypoints, %.2fs total\n",
                tracking_mode == RAMSETE ? "RAMSETE" : "LTV",
                tracked_waypoints.size(),
                tracked_total_time_sec);
  }

  drive_mode_set(tracking_mode);
}

void Drive::tracked_task(bool use_ltv) {
  if (!tracked_mode_active()) {
    drive_mode_set(DISABLE, true);
    return;
  }

  const double elapsedSec = std::max(0.0, static_cast<double>(pros::millis() - tracked_start_ms) / 1000.0);
  const double clampedTime = std::clamp(elapsedSec, 0.0, tracked_total_time_sec);

  tracked_sample desired = tracked_samples.front();
  if (tracked_samples.size() == 1 || tracked_total_time_sec <= 0.0) {
    tracked_current_sample_index = 0;
  } else {
    const double position = (clampedTime / tracked_total_time_sec) * static_cast<double>(tracked_samples.size() - 1);
    const int index = std::clamp(static_cast<int>(std::floor(position)), 0, static_cast<int>(tracked_samples.size() - 1));
    const double alpha = position - static_cast<double>(index);
    tracked_current_sample_index = index;

    if (index + 1 < static_cast<int>(tracked_samples.size())) {
      const tracked_sample& a = tracked_samples[static_cast<std::size_t>(index)];
      const tracked_sample& b = tracked_samples[static_cast<std::size_t>(index + 1)];
      desired = a;
      desired.x_m = a.x_m + (b.x_m - a.x_m) * alpha;
      desired.y_m = a.y_m + (b.y_m - a.y_m) * alpha;
      desired.theta_rad = wrapRadians(a.theta_rad + angleDiffRad(b.theta_rad, a.theta_rad) * alpha);
      desired.linear_velocity_mps = a.linear_velocity_mps + (b.linear_velocity_mps - a.linear_velocity_mps) * alpha;
      desired.angular_velocity_rps = a.angular_velocity_rps + (b.angular_velocity_rps - a.angular_velocity_rps) * alpha;
      desired.linear_accel_mps2 = a.linear_accel_mps2 + (b.linear_accel_mps2 - a.linear_accel_mps2) * alpha;
      desired.pose_target = toEzPose(desired.x_m, desired.y_m, desired.theta_rad);
    } else {
      desired = tracked_samples.back();
      tracked_current_sample_index = static_cast<int>(tracked_samples.size() - 1);
    }
  }

  const InternalPose current = toInternalPose(odom_pose_get());
  const double dx = desired.x_m - current.x_m;
  const double dy = desired.y_m - current.y_m;
  const double cosTheta = std::cos(current.theta_rad);
  const double sinTheta = std::sin(current.theta_rad);
  const double errorX = cosTheta * dx + sinTheta * dy;
  const double errorY = -sinTheta * dx + cosTheta * dy;
  const double errorTheta = angleDiffRad(desired.theta_rad, current.theta_rad);

  double linearVelocity = desired.linear_velocity_mps;
  double angularVelocity = desired.angular_velocity_rps;

  if (use_ltv && !tracked_ltv_gains.empty()) {
    const tracked_gain gain = tracked_ltv_gains[static_cast<std::size_t>(std::clamp(
        tracked_current_sample_index,
        0,
        static_cast<int>(tracked_ltv_gains.size() - 1)))];
    const double deltaV =
        -(gain.k00 * errorX + gain.k01 * errorY + gain.k02 * errorTheta);
    const double deltaOmega =
        -(gain.k10 * errorX + gain.k11 * errorY + gain.k12 * errorTheta);
    linearVelocity += deltaV;
    angularVelocity += deltaOmega;
  } else {
    const double k =
        2.0 * ramsete_zeta *
        std::sqrt(desired.angular_velocity_rps * desired.angular_velocity_rps
                  + ramsete_beta * desired.linear_velocity_mps * desired.linear_velocity_mps);
    linearVelocity =
        desired.linear_velocity_mps * std::cos(errorTheta) + k * errorX;
    angularVelocity =
        desired.angular_velocity_rps + k * errorTheta
        + ramsete_beta * desired.linear_velocity_mps * sinc(errorTheta) * errorY;
  }

  linearVelocity = clampAbs(linearVelocity, CONFIG::MAX_SPEED.convert(mps));
  angularVelocity = clampAbs(angularVelocity, CONFIG::MAX_ANGULAR_VEL.convert(radps));

  const double trackWidthM = drive_width_get() * kInToM;
  const double leftWheelMps = linearVelocity - angularVelocity * trackWidthM * 0.5;
  const double rightWheelMps = linearVelocity + angularVelocity * trackWidthM * 0.5;
  const double maxWheelMps =
      ((CARTRIDGE / std::max(0.001, RATIO)) * (WHEEL_DIAMETER * M_PI * kInToM)) / 60.0;

  double leftCommand = maxWheelMps > kTrackingPathEpsilonM
      ? (leftWheelMps / maxWheelMps) * 127.0
      : 0.0;
  double rightCommand = maxWheelMps > kTrackingPathEpsilonM
      ? (rightWheelMps / maxWheelMps) * 127.0
      : 0.0;

  const double fasterSide = std::max(std::fabs(leftCommand), std::fabs(rightCommand));
  if (fasterSide > tracked_speed_cap && fasterSide > 1e-6) {
    const double scale = static_cast<double>(tracked_speed_cap) / fasterSide;
    leftCommand *= scale;
    rightCommand *= scale;
  }

  if (tracked_slew_on) {
    const double step = std::max(CONFIG::AUTON_DRIVE_SLEW_STEP, CONFIG::AUTON_TURN_SLEW_STEP);
    leftCommand = std::clamp(leftCommand, tracked_last_left_cmd - step, tracked_last_left_cmd + step);
    rightCommand = std::clamp(rightCommand, tracked_last_right_cmd - step, tracked_last_right_cmd + step);
  }

  tracked_last_left_cmd = leftCommand;
  tracked_last_right_cmd = rightCommand;

  if (drive_toggle) {
    private_drive_set(static_cast<int>(std::lround(leftCommand)),
                      static_cast<int>(std::lround(rightCommand)));
  }
}

void Drive::ramsete_task() {
  tracked_task(false);
}

void Drive::ltv_task() {
  tracked_task(true);
}
