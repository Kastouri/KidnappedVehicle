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
  if (is_initialized == false){
    num_particles = 100;  // TODO: Set the number of particles
      
      // standard diviations
      double std_x = std[0];
      double std_y = std[1];
      double std_theta = std[2];
      
      // instantiate random engine
        std::default_random_engine gen;
        // define normal distributions
        normal_distribution<double> dist_x(x, std_x);
        normal_distribution<double> dist_y(y, std_y);
        normal_distribution<double> dist_theta(theta, std_theta);

      std::cout << "Initializing Particles ..." << std::endl;
      for (int i=0; i < num_particles; ++i){
        Particle nparticle;
        nparticle.id = i;
        nparticle.x = dist_x(gen);  // take sample of normal distribution and store to Particle
        nparticle.y = dist_y(gen);
        nparticle.theta = dist_theta(gen); 
        nparticle.weight = 1.0;
        std::cout << "  Particle " << nparticle.id << ": x = " << nparticle.x << " y = " << nparticle.y << std::endl; 
        
        // save partile in filter
        particles.push_back(nparticle);
      }
      
    // set initialization flag
    is_initialized = true;
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
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];
  
  // instantiate random engine
  std::default_random_engine gen;
  // define normal distributions
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);

  // for each particle
  for (int i=0; i< num_particles; ++i){
    double x, y, theta;
    double x_f, y_f,theta_f;

    // TODO: add random noise to velocity and yaw rate
     // standard diviations

    x = particles[i].x;
    y = particles[i].y;
    theta = particles[i].theta;
    if (abs(yaw_rate) < 0.00000001) { // avoid zero division : linear motion
      // yaw_rate = (yaw_rate > 0) ?  0.0000000001: -0.0000000001;
      x_f = x + velocity * delta_t * cos(theta);
      y_f = y + velocity * delta_t * sin(theta);
    } else
    {   // bike model
      x_f = x + velocity/yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
      y_f = y + velocity/yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
      theta_f = theta + yaw_rate * delta_t;
    
    }
          
    // updtae particles
    particles[i].x= x_f + dist_x(gen);
    particles[i].y = y_f + dist_y(gen);
    particles[i].theta = theta_f + dist_theta(gen);
    
    // print initialization information
    std::cout << "Inputs: theta=" << velocity << " yaw_rate=" << yaw_rate << std::endl;
    std::cout << "prediction x="<< x_f << " y=" << y_f << " theta=" << theta_f << std::endl;
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
  
  // land mark standard deviation
  double std_x_lm = std_landmark[0];
  double std_y_lm = std_landmark[1];
  
  // sum the weights
  double weights_sum = 0.0;

  for (int p = 0; p < particles.size(); ++p){  // for each particle

    // paticles position
    Particle particle = particles[p];
    double theta = particles[p].theta;
    double x_p = particles[p].x;
    double y_p = particles[p].y;
    
    // neu weight
    double weight = 1.0;  // this variable will hold the product calculating the weight 
    //
    if (observations.size() == 0) {
      std::cout << "WARNING: no observations" << std::endl;
      continue;
    }
    for(int i = 0; i < observations.size(); i++){  // for each observation

      double x_obs, y_obs;  // coordinates of the observation in the car's cooridinate system
      double x_obs_map, y_obs_map; // coordinates of the observation in the map's cooridinate system
      LandmarkObs obs_map;  // hold one observation given in the maps cooridinate system 
      
      x_obs = observations[i].x;
      y_obs = observations[i].y;
      
      // transform measurement from the car's to the map's coordinate system 
      x_obs_map = x_p + cos(theta) * x_obs - sin(theta) * y_obs;
      y_obs_map = y_p + sin(theta) * x_obs + cos(theta) * y_obs;
      
      // skip observation if it is outside the sensor range of the particle
      if (dist(x_obs_map,y_obs_map,x_p,y_p) > sensor_range) { 
        std::cout << "WARNING: observation is not in the range of particle" << std::endl;
        continue;
      } 
      
      //std::cout << "Cordinates in the Cars Coordinates System: x=" << x_obs << " y=" << y_obs << std::endl;
      //std::cout << "Cordinates in the Cars Coordinates System: x=" << x_obs_map << " y=" << y_obs_map << std::endl;
      
      
      // map the predicted observation to the closest landmark
      double best_dist = 100000; // initialize distance to a big number
      double best_x, best_y;
      int best_id;

      for (int lm_i = 0 ; lm_i < map_landmarks.landmark_list.size(); ++lm_i){

        // calculate distance
        double x_land = map_landmarks.landmark_list[lm_i].x_f;
        double y_land = map_landmarks.landmark_list[lm_i].y_f;
        
        //std::cout << "Coordinates of predicted measurement in maps system: x=" << x_land<< " y=" << y_land << std::endl;
        double distance = dist(x_land, y_land, x_obs_map, y_obs_map); 
        
        // if distance is smaller than the best distance update the best assossiation
        if (distance < best_dist){
          best_dist = distance; 
          best_x = x_land;
          best_y = y_land;
        }
      }
      //if (best_dist < sensor_range){
        // take best measurement into the product
        double obs_prob = exp_w(x_obs_map, y_obs_map, best_x, best_y, std_x_lm,  std_y_lm);
        obs_prob = obs_prob / (2.0 * M_PI * std_x_lm * std_y_lm); 
        if (obs_prob < 0.000001) {obs_prob = 0.000001;}
        weight = weight * obs_prob;
      //}
      
    }
    
    // add weight to sum
    weights_sum += weight;
    // set particles new weight
    if (weight < 0.0000001) { weight = 0.0000001;}
    particles[p].weight = weight;
    std::cout << "Particle "<< p << "'s neu weight=" << weight << std::endl;
  }
  // normalize the weights bei their sum
  //for (int p = 0; p < particles.size(); ++p) {
    //particles[p].weight = particles[p].weight / weights_sum; 
  //}
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  /**
   * using Sebastian's resampling wheel
   */
    
  vector<Particle> particles_n;  // new set of particles
  vector<double> weights;

  // sum the weights
  double weights_sum = 0.0;
  for (int p = 0; p < particles.size(); ++p) {
    weights.push_back(particles[p].weight);
  }

  // find the maximum weight
  const double max_weight = *std::max_element(weights.begin(), weights.end());

  std::default_random_engine gen;
  
  // draw discrete index
  std::uniform_int_distribution<int> dist_index(0, num_particles-1);
  int index = dist_index(gen);
  
  
  double beta = 0.0;
  // continious distribution for beta
  

  for (int p = 0; p < num_particles; ++p){
    std::uniform_real_distribution<double> dist_beta(0.0 , max_weight * 2.0);
    beta += dist_beta(gen);
    while (beta > particles[index].weight){
      beta -= particles[index].weight;
      index = (index + 1) % num_particles;
    }
    particles_n.push_back(particles[index]);
  }
  particles = particles_n;
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