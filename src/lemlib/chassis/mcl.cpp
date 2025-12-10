#include <math.h>
#include "pros/rtos.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/mcl.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include <random>
#include <vector>
#include <cstdlib>

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

lemlib::Pose lemlib::Particle::expectedPoint(lemlib::Beam beam) {
        float global_theta = pose.theta + beam.angle;
        return lemlib::Pose(
                pose.x + beam.distance * cos(global_theta),
                pose.y + beam.distance * sin(global_theta),
                global_theta
        );
}

float lemlib::Particle::distanceToWall(lemlib::Pose pose) {
        // this approximates distance from point to wall
        // **not as accurate as raycasting**
        // but works fine in my experience

        // 72 is half the field size in inches

        float d1 = abs((pose.x - 72) / cos(pose.theta));
        float d2 = abs((pose.x + 72) / cos(pose.theta));
        float d3 = abs((pose.y - 72) / cos(pose.theta));
        float d4 = abs((pose.y + 72) / cos(pose.theta));

        // find the min
        float dmin = d1;
        if (d2 < dmin) dmin = d2;
        if (d3 < dmin) dmin = d3;
        if (d4 < dmin) dmin = d4;
        return dmin;
}

float lemlib::MCLConfigs::gaussian(float x) {
        return GAUSSIAN_FACTOR * exp(-0.5 * (x / GAUSSIAN_STDEV)*(x / GAUSSIAN_STDEV)) / (GAUSSIAN_STDEV * sqrt(2 * std::numbers::pi));
}

void lemlib::Particle::updateWeight(std::vector<Beam> &beams) {
        float sum = 0;
        for (auto beam : beams) {
                sum += mcl_configs.gaussian(distanceToWall(expectedPoint(beam)));
        }
        weight = sum;
}
