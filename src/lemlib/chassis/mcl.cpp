#include <math.h>
#include "pros/rtos.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/mcl.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include <vector>

// The particles in the simulation
std::vector<lemlib::Particle> particles;

// A temporary array for particles whilst resampling
std::vector<lemlib::Particle> resamp_particles;

// The global MCL configs
lemlib::MCLConfigs mcl_configs(0, 0, 0, 0, 0);

void initializeParticles() {
        for (auto &p : particles)
                p = lemlib::Particle(0, 0, 0, -1);
}

void lemlib::Particle::updateDeltaNoise(lemlib::Pose delta_pose) {
        pose.x += mcl_configs.XY_NOISE * 2.0f * (((float)rand())/RAND_MAX - 0.5) + delta_pose.x;
        pose.y += mcl_configs.XY_NOISE * 2.0f * (((float)rand())/RAND_MAX - 0.5) + delta_pose.y;
        pose.theta += mcl_configs.THETA_NOISE * 2.0f * (((float)rand())/RAND_MAX - 0.5) + delta_pose.theta;
}
