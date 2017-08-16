/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <map>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	const double std_x = std[0];
	const double std_y = std[1];
	const double std_theta = std[2];

	default_random_engine gen;
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	num_particles = 10;
	for (int i = 0; i < num_particles; i++) {
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1;
		weights.push_back(p.weight);
		particles.push_back(p);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double dt, double std_pos[], double v,
		double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	const double std_x = std_pos[0];
	const double std_y = std_pos[1];
	const double std_theta = std_pos[2];

	default_random_engine rand_engine;

	for (auto &p : particles) {

		double new_x, new_y, new_theta;
		if (fabs(yaw_rate) > 1e-5) {
			double c = v / yaw_rate;
			new_x = p.x + c * (sin(p.theta + yaw_rate * dt) - sin(p.theta));
			new_y = p.y + c * (cos(p.theta) - cos(p.theta + yaw_rate * dt));
			new_theta = p.theta + yaw_rate * dt;
		} else {
			double c = v * dt;
			new_x = p.x + c * cos(p.theta);
			new_y = p.y + c * sin(p.theta);
			new_theta = p.theta;
		}

		normal_distribution<double> dist_x(new_x, std_x);
		normal_distribution<double> dist_y(new_y, std_y);
		normal_distribution<double> dist_theta(new_theta, std_theta);

		p.x = dist_x(rand_engine);
		p.y = dist_y(rand_engine);
		p.theta = dist_theta(rand_engine);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
		std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (auto& obs : observations) {
		double minDist = std::numeric_limits<float>::max();
		for (const auto& pred : predicted) {
			double distance = dist(obs.x, obs.y, pred.x, pred.y);
			if (minDist > distance) {
				minDist = distance;
				obs.id = pred.id;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	const double sig_landmark_x = std_landmark[0];
	const double sig_landmark_y = std_landmark[1];

	weights.clear();

	for (auto& p: particles) {

		p.weight = 1.0;

		map<int, LandmarkObs> landmarksInRange;
		for (const auto& landmark : map_landmarks.landmark_list) {
			if ((landmark.x_f >= p.x-sensor_range && landmark.x_f <= p.x+sensor_range) &&
					(landmark.y_f >= p.y-sensor_range && landmark.y_f <= p.y+sensor_range)) {
				LandmarkObs pred;
				pred.id = landmark.id_i;
				pred.x = landmark.x_f;
				pred.y = landmark.y_f;
				landmarksInRange[pred.id] = pred;
			}
		}

		vector<LandmarkObs> obsMapList;
		for (const auto& obs: observations) {
			double glob_obs_x = obs.x*cos(p.theta) - obs.y*sin(p.theta) + p.x;
			double glob_obs_y = obs.x*sin(p.theta) + obs.y*cos(p.theta) + p.y;

			LandmarkObs obsMap;
			obsMap.id = -1;
			obsMap.x = glob_obs_x;
			obsMap.y = glob_obs_y;

			double minDist = std::numeric_limits<float>::max();
			for (const auto &it: landmarksInRange) {
				auto &pred = it.second;
				double distance = dist(obsMap.x, obsMap.y, pred.x, pred.y);
				if (distance < minDist) {
					minDist = distance;
					obsMap.id = pred.id;
				}
			}

			obsMapList.push_back(obsMap);
		}

		vector<int> associations;
		vector<double> senseX;
		vector<double> senseY;
		for (const auto& obs : obsMapList) {
			LandmarkObs& association = landmarksInRange.find(obs.id)->second;
			associations.push_back(association.id);
			senseX.push_back(association.x);
			senseY.push_back(association.y);

			double diffX = association.x - obs.x;
			double diffY = association.y - obs.y;
			double p1 = (diffX * diffX) / (2 * sig_landmark_x * sig_landmark_x);
			double p2 = (diffY * diffY) / (2 * sig_landmark_y * sig_landmark_y);

			double weight = exp(-(p1 + p2)) / (2 * M_PI * sig_landmark_x * sig_landmark_y);
			p.weight *= weight;
		}

		SetAssociations(p, associations, senseX, senseY);
		weights.push_back(p.weight);
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine rand_engine;

	std::vector<Particle> newParticles;
	std::discrete_distribution<> d(weights.begin(), weights.end());

	for (int i = 0; i < num_particles; i++) {
		int idx = d(rand_engine);
		newParticles.push_back(particles[idx]);
	}
	particles = newParticles;
}

Particle ParticleFilter::SetAssociations(Particle particle,
		std::vector<int> associations, std::vector<double> sense_x,
		std::vector<double> sense_y) {
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseX(Particle best) {
	vector<double> v = best.sense_x;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseY(Particle best) {
	vector<double> v = best.sense_y;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}
