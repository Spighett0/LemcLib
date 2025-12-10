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

// temporary vectors reserved to save time
std::vector<float> particle_sum_weights;
std::vector<float> particle_offsets;
std::vector<lemlib::Particle> particle_resamp;

// The global MCL configs
lemlib::MCLConfigs mcl_configs(0, 0, 0, 0, 0);

// The average pose of all the particles
lemlib::Pose avg_pose = lemlib::Pose(0, 0, 0);

void lemlib::initMCL() {
        particle_resamp.reserve(mcl_configs.N_PARTICLES);
        particle_sum_weights.reserve(mcl_configs.N_PARTICLES);
        particle_offsets.reserve(mcl_configs.N_PARTICLES);

        particles.reserve(mcl_configs.N_PARTICLES);
        for (int i = 0; i < mcl_configs.N_PARTICLES; i++)
                particles.push_back(lemlib::Particle(0, 0, 0, -1));
}

lemlib::Beam lemlib::Beamer::getBeam() {
        // get the distance sensor reading in inches
        float distance = ((float)sensor.get_distance()) / 25.4;

        // unit vector in beam direction
        float ux = cos(angle);
        float uy = sin(angle);

        // correct distance is
        float distance_correct = distance + (x_offset * ux + y_offset * uy);

        return Beam(angle, distance_correct);
}

void updateStep(lemlib::Pose delta_pose) {
        for (auto &particle : particles)
                particle.updateDeltaNoise(delta_pose);
}

// not a single clue how this works, I just listened to claude, this WILL NOT
// work and is NOT tested
void resampleStep(std::vector<lemlib::Beam> &beams) {
        // make sure to reset the pre-allocated stuff
        particle_sum_weights.clear();
        particle_offsets.clear();
        particle_resamp.clear();

        // calculate weights for each particle
        // (weights show how much it matches a sensor reading)
        // 
        // and create a cumulative sum of the weights
        // (creates a weight ladder where high weights take up more space)
        float weights_sum = 0;
        for (auto &particle : particles) {
                particle.updateWeight(beams);
                weights_sum += particle.weight;
                particle_sum_weights.push_back(weights_sum);
        }

        // create evenly-spaced selection points (Low Variance Resampling)
        float random_offset = (((float)rand())/RAND_MAX)/mcl_configs.N_PARTICLES;
        for (int i = 0; i < mcl_configs.N_PARTICLES; i++) {
                // Create evenly-spaced offsets
                float offset = random_offset + (float)i / mcl_configs.N_PARTICLES;

                // Scale offset to match total weight
                offset = offset * weights_sum;

                particle_offsets.push_back(offset);
        }

        // select new particles based off offsets
        for (float offset : particle_offsets) {
                // For this offset, find which particle it selects

                for (int j = 0; j < mcl_configs.N_PARTICLES; j++) {
                    // Check if this particle's cumulative weight is >= offset
                    if (particle_sum_weights[j] >= offset) {
                        // Found it! This particle gets selected
                        particle_resamp.push_back(particles[j]);  // Copy the particle
                        break;  // Move to next offset
                    }
                }
        }

        // replace the old particles with the new ones
        std::vector<lemlib::Particle> tmp;
        tmp = particles;
        particles = particle_resamp;
        particle_resamp = particles;

        // reset all the weights to equal and also sum the poses
        lemlib::Pose sum_pose = lemlib::Pose(0, 0, 0);
        for (auto &p : particles) {
                p.weight = 1.0 / mcl_configs.N_PARTICLES;
                sum_pose.x = p.pose.x;
                sum_pose.y = p.pose.y;
                sum_pose.theta = p.pose.theta;
        }
        // calculate the average pose
        avg_pose = lemlib::Pose(
                sum_pose.x / mcl_configs.N_PARTICLES,
                sum_pose.y / mcl_configs.N_PARTICLES,
                sum_pose.theta / mcl_configs.N_PARTICLES
        );
}

// Runs MCL
lemlib::Pose lemlib::runMCL(std::vector<lemlib::Beam> &beams, lemlib::Pose delta_pose) {
        updateStep(delta_pose);
        resampleStep(beams);
        return avg_pose;
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
        for (auto beam : beams)
                sum += mcl_configs.gaussian(distanceToWall(expectedPoint(beam)));
        weight = sum;
}
