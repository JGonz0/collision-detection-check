#include "obs_definition.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numeric>

polygon createPol() {

  polygon P1;
  P1.push_back(vertex(-1.0, -1.0));
  P1.push_back(vertex(-0.5, 0.0));
  P1.push_back(vertex(0.0, 1.0));

  return P1;
}

obstacles createObs() {

  obstacles ObsStruct;
  polygon P1;
  P1.push_back(vertex(-0.5, -0.5));
  P1.push_back(vertex(-0.0, 0.0));
  P1.push_back(vertex(0.5, -0.5));
  ObsStruct.push_back(P1);

  polygon P2;
  P2.push_back(vertex(-0.5, 0.5));
  P2.push_back(vertex(0.0, 0.0));
  P2.push_back(vertex(0.5, 0.5));
  ObsStruct.push_back(P2);

  polygon P3;
  P3.push_back(vertex(-0.25, 0.25));
  P3.push_back(vertex(0.25, 0.25));
  P3.push_back(vertex(0.25, -0.25));
  P3.push_back(vertex(-0.25, -0.25));
  ObsStruct.push_back(P3);

  polygon P4;
  P4.push_back(vertex(0.5, 1.0));
  P4.push_back(vertex(1.0, 1.0));
  P4.push_back(vertex(1.0, 0.0));
  ObsStruct.push_back(P4);

  polygon P5;
  P5.push_back(vertex(0.5, -1.0));
  P5.push_back(vertex(1.0, -1.0));
  P5.push_back(vertex(1.0, 0.0));
  ObsStruct.push_back(P5);

  polygon P6;
  P6.push_back(vertex(-0.5, 1.0));
  P6.push_back(vertex(-1.0, 1.0));
  P6.push_back(vertex(-1.0, 0.0));
  ObsStruct.push_back(P6);

  polygon P7;
  P7.push_back(vertex(-0.5, -1.0));
  P7.push_back(vertex(-1.0, -1.0));
  P7.push_back(vertex(-1.0, 0.0));
  ObsStruct.push_back(P7);

  return ObsStruct;
}

polygon centroid_calculation(obstacles &obs) {

  polygon centroids{};
  vertex current{0.0, 0.0};
  // o_itr stands for obstacles' iterator,
  // p_itr stands for polygon's iterator

  for (auto o_itr = obs.begin(); o_itr != obs.end(); ++o_itr) {

    double x_coord = 0.0, y_coord = 0.0;

    for (auto p_itr = (*o_itr).begin(); p_itr != (*o_itr).end(); ++p_itr) {
      x_coord += p_itr->first;
      y_coord += p_itr->second;
    }
    x_coord = x_coord / (*o_itr).size();
    y_coord = y_coord / (*o_itr).size(); // Avg
    current.first = x_coord, current.second = y_coord;

    centroids.push_back(current);
  }

  return centroids;
}

// Used for collision detection, iterate only thru near obstacles
obstacles island_definition(vertex center, obstacles &obs, polygon &centroids,
                            double radius) {

  obstacles valid_obs{};
  double distance = 0.0;
  uint obs_idx = 0;

  for (auto p_itr = centroids.begin(); p_itr != centroids.end(); ++p_itr) {
    distance = std::pow(std::pow(center.first - p_itr->first, 2.0) +
                            std::pow(center.second - p_itr->second, 2),
                        0.5);

    if (distance < radius) {
      valid_obs.push_back(obs[obs_idx]);
    }

    obs_idx++;
  }

  return valid_obs;
}

bool SAT_collision(polygon &P1, obstacles &obs) {

  // Clear criteria
  uint OBS_NUMBER = obs.size();
  uint OBS_PASS = 0;

  // Iterator definition
  obstacles::iterator o_itr;
  polygon::iterator p_itr;
  polygon::iterator main_itr;

  // Extreme definitions
  // TODO
  double P1_max = NAN, P1_min = NAN, P2_max = NAN, P2_min = NAN;

  // Vector of perpendicular vectors fe edge
  std::vector<vertex> P1_vec;
  std::vector<vertex> P_vec;
  vertex tmp{0.0, 0.0};
  double aux;

  // Stop before last value
  for (main_itr = P1.begin(); main_itr != P1.end() - 1; ++main_itr) {
    tmp.second = (main_itr + 1)->first - main_itr->first;
    tmp.first = -((main_itr + 1)->second - main_itr->second);
    P1_vec.push_back(tmp);
  }

  // Handle last element
  tmp.second = P1.begin()->first - (P1.end() - 1)->first;
  tmp.first = -(P1.begin()->second - (P1.end() - 1)->second);
  P1_vec.push_back(tmp);

  // Loop for obstacle's polygons
  // Here we start with the collision check procedure
  std::vector<vertex> P2_vec;
  polygon P2;

  for (o_itr = obs.begin(); o_itr != obs.end(); ++o_itr) {

    // Get the current obstacle
    P2 = *o_itr;

    // Stop before last value
    for (main_itr = P2.begin(); main_itr != P2.end() - 1; ++main_itr) {
      tmp.second = (main_itr + 1)->first - main_itr->first;
      tmp.first = -((main_itr + 1)->second - main_itr->second);
      P2_vec.push_back(tmp);
    }

    // Handle last element
    tmp.second = P2.begin()->first - (P2.end() - 1)->first;
    tmp.first = -(P2.begin()->second - (P2.end() - 1)->second);
    P2_vec.push_back(tmp);

    // Copy OG P1_vec
    P_vec = P1_vec;
    P_vec.insert(P_vec.end(), P2_vec.begin(), P2_vec.end());

    // Project all vertices in all perpendicular vectors
    for (p_itr = P_vec.begin(); p_itr != P_vec.end(); ++p_itr) {

      aux = 0.0;
      P1_max = NAN, P1_min = NAN;
      P2_max = NAN, P2_min = NAN;

      // For vertices in first polygon
      for (main_itr = P1.begin(); main_itr != P1.end(); ++main_itr) {

        aux = (main_itr->first * p_itr->first) +
              (main_itr->second * p_itr->second);

        if (P1_max != P1_max || P1_max < aux) {
          P1_max = aux;
        }
        if (P1_min != P1_min || P1_min > aux) {
          P1_min = aux;
        }
      }

      // For vertices in second polygon
      for (main_itr = P2.begin(); main_itr != P2.end(); ++main_itr) {
        aux = (main_itr->first * p_itr->first) +
              (main_itr->second * p_itr->second);

        if (P2_max != P2_max || P2_max < aux) {
          P2_max = aux;
        }
        if (P2_min != P2_min || P2_min > aux) {
          P2_min = aux;
        }
      }

      if ((P1_min < P2_max && P1_min > P2_min) ||
          (P2_min < P1_max && P2_min > P1_min)) {
        continue;
      }
      OBS_PASS++;
      break;
    }

    // Reset vectors
    P_vec.clear();
    P2_vec.clear();
  }

  if (OBS_PASS == OBS_NUMBER) {
    return false;
  }
  return true;
}

// SAT algorithm between circle and polygon
bool SAT_collision_circle(vertex center, double radius, obstacles &obs) {

  // Clear criteria
  uint OBS_NUMBER = obs.size();
  uint OBS_PASS = 0;

  // Iterator definition
  obstacles::iterator o_itr;
  polygon::iterator p_itr;
  polygon::iterator main_itr;

  // Extreme definitions
  // TODO
  double P1_max = NAN, P1_min = NAN, P2_max = NAN, P2_min = NAN;

  // Find nearest edge to the center or the circle
  // Vector of perpendicular vectors fe edge
  std::vector<vertex> P1_vec;
  std::vector<vertex> P_vec;
  vertex tmp{0.0, 0.0};
  vertex axis{0.0, 0.0};
  double aux;
  double min_val = NAN;

  // For new axis normalization
  vertex axis_norm{0.0, 0.0};

  // Loop for obstacle's polygons
  // Here we start with the collision check procedure
  std::vector<vertex> P2_vec;
  polygon P2;

  for (auto o_itr = obs.begin(); o_itr != obs.end(); o_itr++) {

    P2 = *o_itr;
    min_val = NAN;

    // Define axis by checking nearest vertex to the center of the circle
    for (auto main_itr = P2.begin(); main_itr != P2.end(); ++main_itr) {
      aux = std::pow(std::pow(main_itr->first - center.first, 2.0) +
                         std::pow(main_itr->second - center.second, 2.0),
                     0.5);

      if (min_val != min_val || aux < min_val) {
        // axis variable will catch the resulting axis
        axis.first = (main_itr->first - center.first);
        axis.second = (main_itr->second - center.second);
        min_val = aux;
      }
    }

    // Stop before last value
    for (main_itr = P2.begin(); main_itr != P2.end() - 1; ++main_itr) {
      tmp.second = (main_itr + 1)->first - main_itr->first;
      tmp.first = -((main_itr + 1)->second - main_itr->second);
      P2_vec.push_back(tmp);
    }

    // Handle last element
    tmp.second = P2.begin()->first - (P2.end() - 1)->first;
    tmp.first = -(P2.begin()->second - (P2.end() - 1)->second);
    P2_vec.push_back(tmp);

    P_vec = polygon{axis};
    P_vec.insert(P_vec.end(), P2_vec.begin(), P2_vec.end());

    // Look for repeated axes
    polygon unique_axes{};
    bool unique_flag = true;

    for (p_itr = P_vec.begin(); p_itr != P_vec.end(); ++p_itr) {
      for (main_itr = p_itr + 1; main_itr != P_vec.end(); ++main_itr) {
        // Cross product f.e. pair
        // Hardcoded threshold
        if (fabs(p_itr->first * main_itr->second -
                 p_itr->second * main_itr->first) < 0.01) {

          unique_flag = false;
          std::cout << "Similar axes found! \n";
          break;
        }
      }

      if (unique_flag) {
        unique_axes.push_back(vertex{p_itr->first, p_itr->second});
      }

      unique_flag = true;
    }

    // Name issue (skill issue)
    P_vec = unique_axes;
    
    // Check collision fe vertex
    for (p_itr = P_vec.begin(); p_itr != P_vec.end(); ++p_itr) {

      P1_max = NAN, P1_min = NAN;
      P2_max = NAN, P2_min = NAN;

      // For vertices in second polygon
      for (main_itr = P2.begin(); main_itr != P2.end(); ++main_itr) {
        aux = (main_itr->first * p_itr->first) +
              (main_itr->second * p_itr->second);

        if (P2_max != P2_max || P2_max < aux) {
          P2_max = aux;
        }
        if (P2_min != P2_min || P2_min > aux) {
          P2_min = aux;
        }
      }

      // For circle
      aux = std::pow(std::pow(p_itr->first, 2.0) + std::pow(p_itr->second, 2.0),
                     0.5);

      axis_norm.first = (p_itr->first / aux);
      axis_norm.second = (p_itr->second / aux);
      // aux = (p_itr->first * center.first) + (p_itr->second *
      // center.second);

      vertex point1{center.first + axis_norm.first * radius,
                    center.second + axis_norm.second * radius};
      vertex point2{center.first - axis_norm.first * radius,
                    center.second - axis_norm.second * radius};

      P1_min = p_itr->first * point1.first + p_itr->second * point1.second;
      P1_max = p_itr->first * point2.first + p_itr->second * point2.second;

      if (P1_min > P1_max) {
        aux = P1_min;
        P1_min = P1_max;
        P1_max = aux;
      }

      if ((P1_min < P2_max && P1_min > P2_min) ||
          (P2_min < P1_max && P2_min > P1_min)) {
        continue;
      }
      OBS_PASS++;
      break;
    }

    // Reset vectors
    P_vec.clear();
    P2_vec.clear();
  }

  if (OBS_PASS == OBS_NUMBER) {
    return false;
  }
  return true;
}
