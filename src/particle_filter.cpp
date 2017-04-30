/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 * 	Modified: Calvenn Tsuu
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

//#include "Eigen/Dense"
//#include <vector>
//#include <assert.h>

#include "particle_filter.h"

using namespace std;

//using Eigen::MatrixXd;
//using Eigen::VectorXd;



void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.or
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	//std::random_device rd;

	//std::default_random_engine gen(rd());
	std::default_random_engine gen;

	num_particles = 500;

	
        //create normal distribution that mean based on x, y and theta
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);


	// Initializes all particles
	for (int i=0; i < num_particles; i++) {
		
		Particle pa;

                pa.id = i;
		pa.x = dist_x(gen);
		pa.y = dist_y(gen);
		pa.theta = dist_theta(gen);
		pa.weight = 1.0;

                //append new particles to the particles list
		particles.push_back(pa);
		weights.push_back(1.0);
		

	}
	//cout << "num_particles=" << particles.size() << endl;
	is_initialized = true;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine

	//std::random_device rd;
	//std::default_random_engine gen(rd());
	std::default_random_engine gen;
	
	// Uncertainty generators for x, y, theta with 0 mean, 40m x20m map area, and 360 deg all around heading
	std::normal_distribution<double> ndist_x(0, std_pos[0]*40);
	std::normal_distribution<double> ndist_y(0, std_pos[1]*20);
	std::normal_distribution<double> ndist_theta(0, std_pos[2]*2*M_PI);

	for (int i=0; i< particles.size(); i++){
		
		if(fabs(yaw_rate) <0 ) {
			// yaw rate equal to 0
			particles[i].x += velocity*delta_t*cos(particles[i].theta);
			particles[i].y += velocity*delta_t*sin(particles[i].theta);
	        }

		else {	// yaw rate != 0
			particles[i].x += velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
                        particles[i].y += velocity/yaw_rate*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
                        particles[i].theta += yaw_rate*delta_t;
		}
		// Add the noise part
		particles[i].x += ndist_x(gen);
		particles[i].y += ndist_y(gen);
		particles[i].theta += ndist_theta(gen);

	}
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particnular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	
	
}



void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html


	// initializing important weights
	double im_weight=1;
	weights.clear();

	// Create vector for landmarks in sensor range
	std::vector<LandmarkObs> in_range;
	// Create vector for tranformed observation from vehicle coordinate to global coordinate
	std::vector<LandmarkObs> tx_global;
	// create temp vector to find nearest landmark neighbor
	std::vector<LandmarkObs> hot_landmarks;
	
	
	// Iterate over all particles
	for (int i=0; i<num_particles; i++){

		// Transform observations from vehicle coordinates to global map coordinates.
		for (size_t t = 0; t < observations.size(); ++t) {
			
			LandmarkObs tx_obs;

      			tx_obs.x =
			observations[t].x * cos(particles[i].theta)-observations[t].y * sin(particles[i].theta) + particles[i].x;
      			tx_obs.y =
        		observations[t].x * sin(particles[i].theta)+observations[t].y * cos(particles[i].theta) + particles[i].y;
						
			tx_global.push_back(tx_obs);
		}

		hot_landmarks = tx_global;

		
		

		// filter out landmarks beyond the sensor range
		for (size_t r = 0; r < map_landmarks.landmark_list.size(); ++r) {
			Map::single_landmark_s landmark = map_landmarks.landmark_list[r];
			
			landmark.x_f = map_landmarks.landmark_list[r].x_f;
			landmark.y_f = map_landmarks.landmark_list[r].y_f; 

			// use helper function calculate distances
			double distance = dist(particles[i].x, particles[i].y, landmark.x_f, landmark.y_f); 
			
			if (distance <= sensor_range) {
				LandmarkObs lmk;
				lmk.id = landmark.id_i;
				lmk.x = landmark.x_f;
				lmk.y = landmark.y_f;
				in_range.push_back(lmk);
			}  
						
		}
		

    				
		//iterate over each tx_global, find nearest neighbor landmark
		for(int m=0; m<tx_global.size(); m++) {
			double min_dist = std::numeric_limits<double>::max();
			LandmarkObs tx_ = tx_global[m];
			
			for(size_t n=0; n<in_range.size(); n++) {
				LandmarkObs r_ = in_range[n];

				double distance = dist(tx_.x, tx_.y, r_.x, r_.y);
			
				if (distance < min_dist) {
					min_dist = distance;
				
					// update the hot landmark list 
					hot_landmarks[m] = in_range[n];
				}
			}
		}


		
    		
		// Using multivariate Gaussian distribution method to find important weight for each particle
		for (int g=0; g<tx_global.size(); g++) {
			// Find the delta between hot landmark and transformed observations
			double x_dev = hot_landmarks[g].x-tx_global[g].x;
			double y_dev = hot_landmarks[g].y-tx_global[g].y;
			double sigx = std_landmark[0];
			double sigy = std_landmark[1];
			
			// calculate the gaussian probablity and multiply by current weight of particle
			double gaussian = (0.5/(M_PI*sigx*sigy))*exp(-((0.5*x_dev*x_dev/sigx*sigx) + (0.5*y_dev*y_dev/sigy*sigy)));
			im_weight *= gaussian;
		}


    		// update particle weight and add weight to weights vector
		particles[i].weight = im_weight;
		weights.push_back(im_weight);

		
		// Reinitializing weight and clearing vectors for next particle
		im_weight=1.0;
		in_range.clear();
		tx_global.clear();
		hot_landmarks.clear();
  	}
}



void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	//std::random_device rd;
	std::default_random_engine gen;

	std::discrete_distribution<int> dd_index(weights.begin(), weights.end());

	// create a bag of temp particles to transfer resampled particles
	std::vector<Particle> resampled_particles;
	resampled_particles.clear();

	for(int i=0; i<num_particles; i++){
        	
        	resampled_particles.push_back(particles[dd_index(gen)]);
	}

	// replace the original particles with survived particles
	particles = resampled_particles;
		
}



void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}


void ParticleFilter::writeBest(Particle best, std::vector<LandmarkObs> noisy_observations, std::string filename, int time_step) {

	// If first time step create txt file
	if(time_step == 0)
	{
		std::ofstream outfile (filename);
		outfile.close();
	}

	// Append best particle's x,y,theta and associated observations to text file
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);

	dataFile << time_step << " " <<best.x << " " << best.y << " " << best.theta << " " << best.weight;
	
	for(int i = 0; i < noisy_observations.size(); i++)
	{
		dataFile << " " << noisy_observations[i].x << " " << noisy_observations[i].y;
	}
	dataFile << "\n";

	dataFile.close();
}


