/**
 * @file telemetry.h
 * Lightweight telemetry output over serial / screen.
 *
 * Controllers (RAMSETE, LTV) call Telemetry::send() each tick with a JSON
 * payload containing time, current pose [x,y,θ], and desired pose.
 * The data is pushed to the terminal via pros::c::fdctl() or printed.
 */
#pragma once

#include <string>
#include <cstdio>
#include "Eigen/Core"
#include "json/json.h"

class Telemetry {
public:
    /**
     * Send a telemetry sample.
     * @param time      seconds since command start
     * @param current   current robot pose  (x, y, θ)
     * @param desired   desired/target pose (x, y, θ)
     */
    static void send(float time,
                     const Eigen::Vector3f& current,
                     const Eigen::Vector3f& desired) {
        JsonBuilder jb;
        jb.beginObject()
          .key("t").value(time)
          .key("cur").beginArray()
              .rawValue(current.x()).rawValue(current.y()).rawValue(current.z())
          .endArray()
          .key("des").beginArray()
              .rawValue(desired.x()).rawValue(desired.y()).rawValue(desired.z())
          .endArray()
          .endObject();

        std::printf("%s\n", jb.str().c_str());
    }

    /** Send a simple key-value telemetry message. */
    static void send(const std::string& key, float value) {
        std::printf("{\"%s\":%.4f}\n", key.c_str(), value);
    }

    /** Send raw JSON string. */
    static void sendRaw(const std::string& json) {
        std::printf("%s\n", json.c_str());
    }
};
