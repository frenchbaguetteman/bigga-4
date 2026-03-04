#pragma once

#include "Eigen/Dense"
#include <string>

namespace BrainScreen {

struct InitViewModel {
    float progress = 0.0f;            // 0..1
    std::string stageTitle = "";     // e.g. "Init localization"
    std::string detail = "";         // e.g. "GPS poll 4/40"
};

struct RuntimeViewModel {
    Eigen::Vector3f pose = Eigen::Vector3f(0, 0, 0); // legacy primary pose
    Eigen::Vector3f pureOdomPose = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f pureMclPose = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f gpsPose = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f combinedPose = Eigen::Vector3f(0, 0, 0);
    std::string auton = "";
    std::string alliance = "";
    std::string status = "";
};

void initialize();
void renderInit(const InitViewModel& vm);
void renderRuntime(const RuntimeViewModel& vm);

}  // namespace BrainScreen
