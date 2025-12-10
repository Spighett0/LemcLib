#pragma once

#include "lemlib/pose.hpp"

namespace lemlib {

// Particle in the MCL
class Particle {
        public:
                lemlib::Pose pose;
                float weight;
        Particle(float x, float y, float theta, float weight)
                : pose(lemlib::Pose(x, y, theta)),
                  weight(weight) {}

        void updateDeltaNoise(lemlib::Pose delta_pose);
};

// A 'Beam' in the MCL
// Essentially a raycast -- a distance with an angle (from the distance sensor)
class Beam {
        float angle;
        float distance;
};

// Configs for MCL
class MCLConfigs {
        public:
                float N_PARTICLES;
                float GAUSSIAN_STDEV;
                float GAUSSIAN_FACTOR;

                float THETA_NOISE;
                float XY_NOISE;
        MCLConfigs(float N_PARTICLES, float GAUSSIAN_STDEV, float GAUSSIAN_FACTOR, float THETA_NOISE, float XY_NOISE)
                : N_PARTICLES(N_PARTICLES),
                  GAUSSIAN_STDEV(GAUSSIAN_STDEV),
                  GAUSSIAN_FACTOR(GAUSSIAN_FACTOR),
                  THETA_NOISE(THETA_NOISE),
                  XY_NOISE(XY_NOISE) {}
};

} // namespace lemlib
