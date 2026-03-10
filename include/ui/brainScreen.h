#pragma once

#include "Eigen/Dense"
#include "autonomous/autons.h"
#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace BrainScreen {

struct InitViewModel {
    float progress = 0.0f;            // 0..1
    std::string stageTitle = "";     // e.g. "Init localization"
    std::string detail = "";         // e.g. "GPS poll 4/40"
};

struct RuntimeViewModel {
    struct DistanceSensorViewModel {
        std::string label = "";
        bool valid = false;
        float rangeM = 0.0f;
        int confidence = -1;
    };

    Eigen::Vector3f pose = Eigen::Vector3f(0, 0, 0); // legacy primary pose
    Eigen::Vector3f pureOdomPose = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f pureMclPose = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f gpsPose = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f combinedPose = Eigen::Vector3f(0, 0, 0);
    bool gpsPoseValid = false;
    float gpsErrorM = -1.0f;
    std::size_t pfActiveSensors = 0;
    std::size_t pfAbsoluteSensors = 0;
    bool pfUsedMeasurements = false;
    bool pfDidResample = false;
    double pfEss = 0.0;
    double pfAverageWeight = 0.0;
    float pfRecoveryFraction = 0.0f;
    std::array<DistanceSensorViewModel, 4> distanceSensors{};
    std::vector<Eigen::Vector2f> pfParticleSample;  // sparse cloud for map UI
    Auton selectedAuton = Auton::NONE;
    std::string auton = "";
    std::string status = "";
};

void initialize();
void renderInit(const InitViewModel& vm);
void renderRuntime(const RuntimeViewModel& vm);

}  // namespace BrainScreen
