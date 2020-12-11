/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  
  // instantiate random engine
  std::default_random_engine gen;
  // standard diviations
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];
  // define normal distributions
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  for (int i=0; i < num_particles; ++i){
    Particle nparticle;
    nparticle.id = i;
    nparticle.x = dist_x(gen);  // take sample of normal distribution and store to Particle
    nparticle.y = dist_y(gen);
    nparticle.theta = dist_theta(gen); 
    // save partile in filter
    particles.push_back(nparticle);
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  
  // for each particle
  for (int i=0; i< num_particles; ++i){
    double x, y, theta;
    double x_f, y_f,theta_f;

    // TODO: add random noise to velocity and yaw rate


    x = particles[i].x;
    y = particles[i].y;
    theta = particles[i].theta;
    if (yaw_rate < 0.0000001) { yaw_rate = 0.0000001;}  // avoid zero division
    x_f = x + velocity/yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
    y_f = y + velocity/yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
    theta_f = theta + yaw_rate* delta_t;

    // updtae particles
    particles[i].x= x_f;
    particles[i].y = y_f;
    particles[theta].theta = theta_f;
  }

}

// void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
//                                      vector<LandmarkObs>& observations) {
//   /**
//    * TODO: Find the predicted measurement that is closest to each 
//    *   observed measurement and assign the observed measurement to this 
//    *   particular landmark.
//    * NOTE: this method will NOT be called by the grading code. But you will 
//    *   probably find it useful to implement this method and use it as a helper 
//    *   during the updateWeights phase.
//    */
//   double distance = 1000;
//   double n_dis;  // will hold the new calculated distance
//   double pred_x, pred_y, obs_x, obs_y;
//   double pred_x_best=0, pred_y_best=0;  // TODO: initialize to one of the predictions instead
//   int pred_id_best=0;
//   int pred_id, obs_id;
//   // for each predicted measurement
//   for (int i=0; i<observations.size(); ++i){
//     LandmarkObs obs_v= observations[i];
//     obs_x = obs_v.x;
//     obs_y = obs_v.y;
//     obs_id = obs_v.id;
//     for (auto pred_v: predicted){ // predicted : I think is the map landmarks. TODO: Know, don't Think
//       pred_x = pred_v.x;
//       pred_y = pred_v.y;
//       pred_id = pred_v.id;
//       n_dis = dist(pred_x, pred_y, obs_x, obs_y);
//       // if new calculated distance is less => new best
//       if (n_dis < distance){
//         distance = n_dis;
//         pred_id_best = pred_id;
//         pred_x_best = pred_x;
//         pred_y_best = pred_y;
//       }
//     }
//     // save the best prediction to the obesrvation vector 
//     obs_v.x = pred_x_best;
//     obs_v.y = pred_y_best;
//     obs_v.id = pred_id_best;
//     observations[i] = obs_v;
//   }
// }

void ParticleFilter::dataAssociation(vector<LandmarkObs> map_landmarks, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  /**
   * This method will assossiate a map landmark to each of the measurement.
   * The assossiation will be achieved by setting the measurement ID to the landmark ID  TODO: is this true???
   */

  double best_dist = 1000; // initialize distance to a big number
  
  //for each measurement
  for (int meas_i = 0; meas_i < observations.size(); ++meas_i){
    // for each landmark
    for (int lm_i = 0 ; lm_i < map_landmarks.size(); ++lm_i){
      // calculate distance
      double distance = dist(map_landmarks[lm_i].x, map_landmarks[lm_i].y, 
                              observations[meas_i].x, observations[meas_i].y ); 
      // if distance is smaller than the best distance update the best assossiation
      if (distance < best_dist){
        best_dist = distance;
        observations[meas_i].id = map_landmarks[lm_i].id; 
      }
    } 
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  // 
  
  for (int p = 0; p < particles.size(); ++p){  // for each particle

    Particle particle = particles[p];
    double theta = particles[p].theta;
    double x_p = particles[p].x;
    double y_p = particles[p].y;
    
    vector<LandmarkObs> predicted_observations;
    
    double weight = 1;  // this variable will hold the product calculating the weight 

    for(int i = 0; i < observations.size(); i++){  // for each observation

      double x_obs, y_obs;  // coordinates of the observation in the car's cooridinate system
      double x_obs_map, y_obs_map; // coordinates of the observation in the map's cooridinate system
      LandmarkObs obs_map;  // hold one observation given in the maps cooridinate system 
      
      x_obs = observations[i].x;
      y_obs = observations[i].y;
      
      // transform measurement from the car's to the map's coordinate system 
      x_obs_map = x_p + cos(theta) * x_obs - sin(theta) * y_obs;
      y_obs_map = x_p + sin(theta) * x_obs + cos(theta) * y_obs;
      
      // pack into predicted observations vector
      obs_map.x = x_obs_map;
      obs_map.y = y_obs_map;
      predicted_observations.push_back(obs_map);

      // map the predicted observation to the closest landmark
      double best_dist = 1000; // initialize distance to a big number
      double best_x, best_y;
      int best_id;
      for (int lm_i = 0 ; lm_i < map_landmarks.landmark_list.size(); ++lm_i){
        // calculate distance
        double distance = dist(map_landmarks.landmark_list[lm_i].x_f, map_landmarks.landmark_list[lm_i].y_f, 
                                observations[i].x, observations[i].y ); 
        // if distance is smaller than the best distance update the best assossiation
        if (distance < best_dist){
          best_dist = distance; 
          best_x = map_landmarks.landmark_list[lm_i].x_f;
          best_y = map_landmarks.landmark_list[lm_i].y_f;
        }
      }

    }
  }


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}