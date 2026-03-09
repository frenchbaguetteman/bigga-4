#include "autonomous/autonCommands.h"

#include "autonomous/chassis.h"
#include "command/command.h"
#include "command/instantCommand.h"
#include "pros/rtos.hpp"

#include <atomic>
#include <cmath>
#include <functional>
#include <memory>

namespace {

constexpr double kMToIn = 39.37007874015748;

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

struct RoutineContext {
    Chassis& chassis;
    Intakes& intakes;
    Lift& lift;
    std::function<Eigen::Vector3f()> poseSource;
};

using RoutineFn = void (*)(RoutineContext&);

double metersToInches(float meters) {
    return static_cast<double>(meters) * kMToIn;
}

ez::pose fieldPoseInches(double xInches, double yInches) {
    return ez::pose{xInches, yInches};
}

ez::pose robotRelativePoint(const Eigen::Vector3f& pose, double forwardIn, double leftIn) {
    const double poseXIn = metersToInches(pose.x());
    const double poseYIn = metersToInches(pose.y());
    const double cosT = std::cos(pose.z());
    const double sinT = std::sin(pose.z());

    return ez::pose{
        poseXIn + forwardIn * cosT - leftIn * sinT,
        poseYIn + forwardIn * sinT + leftIn * cosT,
    };
}

void intakeTimed(Intakes& intakes, int voltage, int milliseconds) {
    intakes.spin(voltage);
    pros::delay(milliseconds);
    intakes.stop();
}

void runNegative1(RoutineContext& ctx) {
    ctx.intakes.spin(127);
    ctx.chassis.pid_odom_set({
        {{-47.24, -23.62}, ez::fwd, 102},
        {{-23.62, -23.62}, ez::fwd, 102},
        {{0.0, -11.81}, ez::fwd, 102},
    }, true);
    ctx.chassis.pid_wait();

    pros::delay(300);
    intakeTimed(ctx.intakes, -127, 500);

    ctx.chassis.pid_turn_set(180.0, 96);
    ctx.chassis.pid_wait();

    ctx.chassis.pid_odom_set(fieldPoseInches(-47.24, -23.62), ez::fwd, 96);
    ctx.chassis.pid_wait();
}

void runPositive1(RoutineContext& ctx) {
    ctx.intakes.spin(127);
    ctx.chassis.pid_odom_set({
        {{47.24, -23.62}, ez::fwd, 102},
        {{23.62, -23.62}, ez::fwd, 102},
        {{0.0, -11.81}, ez::fwd, 102},
    }, true);
    ctx.chassis.pid_wait();

    pros::delay(300);
    intakeTimed(ctx.intakes, -127, 500);

    ctx.chassis.pid_turn_set(0.0, 96);
    ctx.chassis.pid_wait();

    ctx.chassis.pid_odom_set(fieldPoseInches(47.24, -23.62), ez::fwd, 96);
    ctx.chassis.pid_wait();
}

void runSkills(RoutineContext& ctx) {
    ctx.intakes.spin(127);

    ctx.chassis.pid_odom_set(fieldPoseInches(-11.81, -55.12), ez::fwd, 127);
    ctx.chassis.pid_wait();
    intakeTimed(ctx.intakes, -127, 400);

    ctx.intakes.spin(127);
    ctx.chassis.pid_odom_set(fieldPoseInches(-11.81, 0.0), ez::fwd, 110);
    ctx.chassis.pid_wait();
    ctx.chassis.pid_odom_set(fieldPoseInches(11.81, 0.0), ez::fwd, 96);
    ctx.chassis.pid_wait();
    intakeTimed(ctx.intakes, -127, 400);

    ctx.lift.moveTo(180.0f);
    pros::delay(150);
    ctx.lift.moveTo(0.0f);

    ctx.chassis.pid_odom_set(fieldPoseInches(47.24, 39.37), ez::fwd, 110);
    ctx.chassis.pid_wait();
    intakeTimed(ctx.intakes, -127, 500);

    ctx.chassis.pid_odom_set(fieldPoseInches(0.0, 0.0), ez::fwd, 96);
    ctx.chassis.pid_wait();
}

void runExampleMove(RoutineContext& ctx) {
    const Eigen::Vector3f startPose =
        ctx.poseSource ? ctx.poseSource() : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    const ez::pose diagonal = robotRelativePoint(startPose, 24.0, 18.0);
    const ez::pose home{metersToInches(startPose.x()), metersToInches(startPose.y())};

    ctx.chassis.pid_drive_set(24.0, 110);
    ctx.chassis.pid_wait();
    ctx.chassis.pid_odom_set(diagonal, ez::fwd, 110);
    ctx.chassis.pid_wait();
    ctx.chassis.pid_odom_set(home, ez::fwd, 110);
    ctx.chassis.pid_wait();
}

void runExampleTurn(RoutineContext& ctx) {
    ctx.chassis.pid_turn_set(90.0, 90);
    ctx.chassis.pid_wait();
    ctx.chassis.pid_turn_set(-90.0, 90);
    ctx.chassis.pid_wait();
    ctx.chassis.pid_turn_set(180.0, 90);
    ctx.chassis.pid_wait();
    ctx.chassis.pid_turn_set(0.0, 90);
    ctx.chassis.pid_wait();
}

void runExamplePath(RoutineContext& ctx) {
    const Eigen::Vector3f start =
        ctx.poseSource ? ctx.poseSource() : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    const ez::pose p0{metersToInches(start.x()), metersToInches(start.y())};
    const ez::pose p1 = robotRelativePoint(start, 18.0, 10.0);
    const ez::pose p2 = robotRelativePoint(start, 36.0, -8.0);
    const ez::pose p3 = robotRelativePoint(start, 48.0, 0.0);

    ctx.intakes.spin(96);
    ctx.chassis.pid_odom_set({
        {p1, ez::fwd, 96},
        {p2, ez::fwd, 96},
        {p3, ez::fwd, 96},
    }, true);
    ctx.chassis.pid_wait();
    intakeTimed(ctx.intakes, -127, 300);
    ctx.chassis.pid_odom_set(p0, ez::fwd, 96);
    ctx.chassis.pid_wait();
}

class AutonRoutineCommand : public Command {
public:
    AutonRoutineCommand(const AutonBuildContext& ctx, RoutineFn routine) {
        auto state = std::make_shared<State>(ctx.intakes, ctx.lift);
        state->poseSource = ctx.poseSource;
        state->routine = routine;
        state->chassis = std::make_shared<Chassis>(
            &ctx.drivetrain,
            ctx.poseSource,
            [state]() {
                return state->cancelRequested.load();
            });
        m_state = std::move(state);
    }

    void initialize() override {
        auto state = m_state;
        state->finished.store(false);
        state->cancelRequested.store(false);

        pros::Task::create([state]() {
            if (state->routine) {
                state->chassis->pid_targets_reset();
                RoutineContext ctx{
                    *state->chassis,
                    state->intakes.get(),
                    state->lift.get(),
                    state->poseSource,
                };
                state->routine(ctx);
            }
            state->chassis->cancel_motion();
            state->finished.store(true);
        }, "auton-routine");
    }

    void execute() override {}

    void end(bool interrupted) override {
        if (!m_state) return;
        if (interrupted) {
            m_state->cancelRequested.store(true);
        }
        if (m_state->chassis) {
            m_state->chassis->cancel_motion();
        }
    }

    bool isFinished() override {
        return !m_state || m_state->finished.load();
    }

private:
    struct State {
        std::shared_ptr<Chassis> chassis;
        std::reference_wrapper<Intakes> intakes;
        std::reference_wrapper<Lift> lift;
        std::function<Eigen::Vector3f()> poseSource;
        RoutineFn routine = nullptr;
        std::atomic<bool> finished{false};
        std::atomic<bool> cancelRequested{false};

        State(Intakes& intakesRef, Lift& liftRef)
            : intakes(intakesRef), lift(liftRef) {}
    };

    std::shared_ptr<State> m_state;
};

std::unique_ptr<Command> makeRoutineCommand(const AutonBuildContext& ctx, RoutineFn routine) {
    return std::make_unique<AutonRoutineCommand>(ctx, routine);
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
            return makeRoutineCommand(ctx, runNegative1);
        case Auton::NEGATIVE_2:
            // TODO: implement runNegative2 — currently falls back to runNegative1
            return makeRoutineCommand(ctx, runNegative1);
        case Auton::POSITIVE_1:
            return makeRoutineCommand(ctx, runPositive1);
        case Auton::POSITIVE_2:
            // TODO: implement runPositive2 — currently falls back to runPositive1
            return makeRoutineCommand(ctx, runPositive1);
        case Auton::EXAMPLE_MOVE:
            return makeRoutineCommand(ctx, runExampleMove);
        case Auton::EXAMPLE_TURN:
            return makeRoutineCommand(ctx, runExampleTurn);
        case Auton::EXAMPLE_PATH:
            return makeRoutineCommand(ctx, runExamplePath);
        case Auton::SKILLS:
            return makeRoutineCommand(ctx, runSkills);
        case Auton::NONE:
        default:
            return std::make_unique<InstantCommand>([]() {});
    }
}

} // namespace autonCommands
