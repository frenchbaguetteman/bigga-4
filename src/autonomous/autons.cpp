#include "autonomous/autonCommands.h"

#include "autonomous/sharedCommands.h"
#include "command/commandGroup.h"
#include "command/instantCommand.h"
#include "command/waitCommand.h"
#include "commands/driveMove.h"
#include "commands/intake/intakeCommand.h"
#include "commands/ramsete.h"
#include "commands/rotate.h"
#include "motionProfiling/motionProfile.h"

#include <cmath>
#include <initializer_list>
#include <memory>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kInToM = 0.0254f;

const std::vector<Auton> kAvailableAutons = {
    Auton::NEGATIVE_1,
    Auton::NEGATIVE_2,
    Auton::POSITIVE_1,
    Auton::POSITIVE_2,
    Auton::EXAMPLE_MOVE,
    Auton::EXAMPLE_TURN,
    Auton::EXAMPLE_PATH,
    Auton::SKILLS,
    Auton::NONE,
};

float wrapRadians(float radians) {
    return std::atan2(std::sin(radians), std::cos(radians));
}

Eigen::Vector3f basePose(const AutonBuildContext& ctx) {
    if (!ctx.poseSource) {
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }

    const Eigen::Vector3f pose = ctx.poseSource();
    if (!std::isfinite(pose.x()) || !std::isfinite(pose.y()) || !std::isfinite(pose.z())) {
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }
    return pose;
}

Eigen::Vector2f robotRelativePoint(const Eigen::Vector3f& pose, float forwardM, float leftM) {
    const float cosT = std::cos(pose.z());
    const float sinT = std::sin(pose.z());
    return Eigen::Vector2f(
        pose.x() + forwardM * cosT - leftM * sinT,
        pose.y() + forwardM * sinT + leftM * cosT);
}

Eigen::Vector3f robotRelativePose(const Eigen::Vector3f& pose,
                                  float forwardM,
                                  float leftM,
                                  float headingDeltaRad) {
    const Eigen::Vector2f point = robotRelativePoint(pose, forwardM, leftM);
    return Eigen::Vector3f(point.x(), point.y(), wrapRadians(pose.z() + headingDeltaRad));
}

MotionProfile buildProfile(std::initializer_list<Eigen::Vector3f> waypoints,
                           float maxVelocityMps,
                           float maxAccelerationMps2) {
    Path path;
    for (const auto& waypoint : waypoints) {
        path.addWaypoint({waypoint.x(), waypoint.y(), waypoint.z()});
    }

    ProfileConstraints constraints{
        QVelocity(maxVelocityMps),
        QAcceleration(maxAccelerationMps2),
    };
    TrapezoidalVelocityProfile velocityProfile(QLength(path.totalLength()), constraints);
    return MotionProfile(path, velocityProfile);
}

std::unique_ptr<Command> makeNegative1(const AutonBuildContext& ctx) {
    MotionProfile profile = buildProfile({
        Eigen::Vector3f(-1.2f, -0.6f, 0.0f),
        Eigen::Vector3f(-0.6f, -0.6f, 0.0f),
        Eigen::Vector3f( 0.0f, -0.3f, 0.5f),
    }, 1.2f, 2.0f);

    return std::make_unique<SequentialCommandGroup>(std::vector<Command*>{
        (new RamseteCommand(ctx.drivetrain, profile, ctx.poseSource))
            ->alongWith(new IntakeSpinCommand(ctx.intakes, 127)),
        new WaitCommand(0.3f),
        shared::outtakeTimed(ctx.intakes, 0.5f),
        new RotateCommand(ctx.drivetrain, kPi, ctx.poseSource),
        new DriveMoveCommand(ctx.drivetrain, Eigen::Vector2f(-1.2f, -0.6f), ctx.poseSource),
    });
}

std::unique_ptr<Command> makePositive1(const AutonBuildContext& ctx) {
    MotionProfile profile = buildProfile({
        Eigen::Vector3f(1.2f, -0.6f, kPi),
        Eigen::Vector3f(0.6f, -0.6f, kPi),
        Eigen::Vector3f(0.0f, -0.3f, kPi - 0.5f),
    }, 1.2f, 2.0f);

    return std::make_unique<SequentialCommandGroup>(std::vector<Command*>{
        (new RamseteCommand(ctx.drivetrain, profile, ctx.poseSource))
            ->alongWith(new IntakeSpinCommand(ctx.intakes, 127)),
        new WaitCommand(0.3f),
        shared::outtakeTimed(ctx.intakes, 0.5f),
        new RotateCommand(ctx.drivetrain, 0.0f, ctx.poseSource),
        new DriveMoveCommand(ctx.drivetrain, Eigen::Vector2f(1.2f, -0.6f), ctx.poseSource),
    });
}

std::unique_ptr<Command> makeSkills(const AutonBuildContext& ctx) {
    auto makeSegment = [&](float x1, float y1, float x2, float y2) -> Command* {
        const float theta = std::atan2(y2 - y1, x2 - x1);
        MotionProfile segment = buildProfile({
            Eigen::Vector3f(x1, y1, theta),
            Eigen::Vector3f(x2, y2, theta),
        }, 1.5f, 2.5f);
        return new RamseteCommand(ctx.drivetrain, segment, ctx.poseSource);
    };

    return std::make_unique<SequentialCommandGroup>(std::vector<Command*>{
        makeSegment(-1.4f, -1.4f, -0.3f, -1.4f)
            ->alongWith(new IntakeSpinCommand(ctx.intakes, 127)),
        shared::outtakeTimed(ctx.intakes, 0.4f),
        makeSegment(-0.3f, -1.4f, -0.3f, 0.0f),
        shared::driveAndIntake(
            ctx.drivetrain,
            ctx.intakes,
            Eigen::Vector2f(0.3f, 0.0f),
            ctx.poseSource),
        shared::outtakeTimed(ctx.intakes, 0.4f),
        shared::liftCycle(ctx.lift, 180.0f, 0.0f),
        makeSegment(0.3f, 0.0f, 1.2f, 1.0f),
        shared::outtakeTimed(ctx.intakes, 0.5f),
        new DriveMoveCommand(ctx.drivetrain, Eigen::Vector2f(0.0f, 0.0f), ctx.poseSource),
    });
}

std::unique_ptr<Command> makeExampleMove(const AutonBuildContext& ctx) {
    const Eigen::Vector3f start = basePose(ctx);
    const Eigen::Vector2f forward = robotRelativePoint(start, 24.0f * kInToM, 0.0f);
    const Eigen::Vector2f diagonal = robotRelativePoint(start, 24.0f * kInToM, 18.0f * kInToM);
    const Eigen::Vector2f home = start.head<2>();

    return std::make_unique<SequentialCommandGroup>(std::vector<Command*>{
        new DriveMoveCommand(ctx.drivetrain, forward, ctx.poseSource),
        new WaitCommand(0.2f),
        new DriveMoveCommand(ctx.drivetrain, diagonal, ctx.poseSource),
        new WaitCommand(0.2f),
        new DriveMoveCommand(ctx.drivetrain, home, ctx.poseSource),
    });
}

std::unique_ptr<Command> makeExampleTurn(const AutonBuildContext& ctx) {
    const Eigen::Vector3f start = basePose(ctx);

    return std::make_unique<SequentialCommandGroup>(std::vector<Command*>{
        new RotateCommand(ctx.drivetrain, wrapRadians(start.z() + 0.5f * kPi), ctx.poseSource),
        new WaitCommand(0.2f),
        new RotateCommand(ctx.drivetrain, wrapRadians(start.z() - 0.5f * kPi), ctx.poseSource),
        new WaitCommand(0.2f),
        new RotateCommand(ctx.drivetrain, wrapRadians(start.z() + kPi), ctx.poseSource),
        new WaitCommand(0.2f),
        new RotateCommand(ctx.drivetrain, start.z(), ctx.poseSource),
    });
}

std::unique_ptr<Command> makeExamplePath(const AutonBuildContext& ctx) {
    const Eigen::Vector3f start = basePose(ctx);
    MotionProfile profile = buildProfile({
        robotRelativePose(start, 0.0f * kInToM, 0.0f * kInToM, 0.0f),
        robotRelativePose(start, 18.0f * kInToM, 10.0f * kInToM, 0.35f),
        robotRelativePose(start, 36.0f * kInToM, -8.0f * kInToM, -0.25f),
        robotRelativePose(start, 48.0f * kInToM, 0.0f * kInToM, 0.0f),
    }, 1.0f, 1.8f);

    return std::make_unique<SequentialCommandGroup>(std::vector<Command*>{
        (new RamseteCommand(ctx.drivetrain, profile, ctx.poseSource))
            ->alongWith(new IntakeSpinCommand(ctx.intakes, 96)),
        new WaitCommand(0.2f),
        shared::outtakeTimed(ctx.intakes, 0.3f),
        new DriveMoveCommand(ctx.drivetrain, start.head<2>(), ctx.poseSource),
    });
}

} // namespace

const std::vector<Auton>& availableAutons() {
    return kAvailableAutons;
}

const char* autonName(Auton auton) {
    switch (auton) {
        case Auton::NEGATIVE_1: return "Negative 1";
        case Auton::NEGATIVE_2: return "Negative 2";
        case Auton::POSITIVE_1: return "Positive 1";
        case Auton::POSITIVE_2: return "Positive 2";
        case Auton::EXAMPLE_MOVE: return "Example Move";
        case Auton::EXAMPLE_TURN: return "Example Turn";
        case Auton::EXAMPLE_PATH: return "Example Path";
        case Auton::SKILLS:     return "Skills";
        case Auton::NONE:       return "None";
    }
    return "Unknown";
}

const char* allianceName(Alliance alliance) {
    switch (alliance) {
        case Alliance::RED:  return "RED";
        case Alliance::BLUE: return "BLUE";
    }
    return "UNKNOWN";
}

namespace autonCommands {

std::unique_ptr<Command> makeAutonCommand(Auton auton, const AutonBuildContext& ctx) {
    switch (auton) {
        case Auton::NEGATIVE_1:
            return makeNegative1(ctx);
        case Auton::NEGATIVE_2:
            return makeNegative1(ctx);
        case Auton::POSITIVE_1:
            return makePositive1(ctx);
        case Auton::POSITIVE_2:
            return makePositive1(ctx);
        case Auton::EXAMPLE_MOVE:
            return makeExampleMove(ctx);
        case Auton::EXAMPLE_TURN:
            return makeExampleTurn(ctx);
        case Auton::EXAMPLE_PATH:
            return makeExamplePath(ctx);
        case Auton::SKILLS:
            return makeSkills(ctx);
        case Auton::NONE:
        default:
            return std::make_unique<InstantCommand>([]() {});
    }
}

} // namespace autonCommands
