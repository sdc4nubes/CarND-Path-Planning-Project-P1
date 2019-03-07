#include "vehicle.hpp"

int VehiclePlanner::lanePlanner(double s, double d, vector<vector<double>> sensor_fusion) {
  int lane = laneCalc(d);
  int new_lane;
  double distance = closestVehicle(s, lane, sensor_fusion, true)[0];
	// Stay in current lane until deciding to change
  curr_lane = lane; 
	// if adequate space in front, stay in lane and go near the speed limit
  if (distance > 20) {
    new_lane = lane;
    target_vehicle_speed = 22.352 - 0.5;
		// Reset average costs for laneCost()
    avg_costs = {0,0,0}; 
    return 0;
  } else {
		// Determine new lane based on cost model
    new_lane = laneCost(s, lane, sensor_fusion);
    vector <double> vehicle = closestVehicle(s, new_lane, sensor_fusion, true);
    target_vehicle_speed = vehicle[1];
  }
  // Return New Lane (0 = stay in lane, -4 = change left, 4 = change right)
  if (new_lane == lane) return 0;
  else if (new_lane < lane) return -4;
  else return 4;
}

int VehiclePlanner::laneCalc(double d) {
  // Check which lane the d-value comes from
  // Left is 0, middle is 1, right is 2
  int lane;
  if (d < 4) lane = 0;
  else if (d < 8) lane = 1;
  else lane = 2;
  return lane;
}

vector<double> VehiclePlanner::closestVehicle(double s, int lane, 
	vector<vector<double>> sensor_fusion, bool direction) {
  double dist = 10000;
	// Set to speed limit in case no vehicles in front
  double velocity = 22.352 - 0.5; 
  double vehicle_s;
  double vehicle_d;
  double vehicle_v;
  int vehicle_lane;
  // Check each vehicle in sensor range
  for(int vehicle = 0; vehicle < sensor_fusion.size(); vehicle++) {
    vehicle_s = sensor_fusion[vehicle][5];
    vehicle_d = sensor_fusion[vehicle][6];
    vehicle_v = sqrt(pow(sensor_fusion[vehicle][3], 2) + pow(sensor_fusion[vehicle][4], 2));
    vehicle_lane = laneCalc(vehicle_d);
    if (vehicle_lane == lane) {
      if (direction == true) {
				// Capture distance and speed of vehicle directly in front
        if (vehicle_s > s and (vehicle_s - s) < dist) {
          dist = vehicle_s - s;
          velocity = vehicle_v;
        }
      } else {
				// Capture distance and speed of vehicle directly behind
        if (s >= vehicle_s and (s - vehicle_s) < dist) {
          dist = s - vehicle_s;
          velocity = vehicle_v;
        }
      }
    }
  }
	// Avoid dividing by zero in laneCost()
  if (dist <= 0) dist = 1.0;
  if (lane == curr_lane and direction == true) curr_lead_vehicle_speed = velocity;
  return {dist, velocity};
}

int VehiclePlanner::laneCost(double s, int lane, vector<vector<double>> sensor_fusion) {
  vector <double> costs = {0,0,0};
  vector <double> front_vehicle;
  vector <double> back_vehicle;
  for (int i = 0; i < 3; i++) {
		// Negative cost (benefit) for staying in lane
    if (i == lane) costs[i] -= 0.5;
    front_vehicle = closestVehicle(s, i, sensor_fusion, true);
    back_vehicle = closestVehicle(s, i, sensor_fusion, false);
		// Negative cost if wide open lane
    if (front_vehicle[0] > 1000 and back_vehicle[0] > 1000) costs[i] -= 5; 
    else {
			// Positive cost for car too close in front
      if (front_vehicle[0] < 10) costs[i] = 5;
			// Positive cost for car too close in back
      if (back_vehicle[0] < 10) costs[i] = 5;
			// Negative cost for large open distance in front of vehicle
      costs[i] -= 1 + (10/(front_vehicle[0]/3));
			// Negative cost for large open distance behind vehicle
      costs[i] -= 1 + (10/(back_vehicle[0]/3)); 
			// Negative cost for faster vehicle in fromt
      costs[i] -= 1 + (10/(front_vehicle[1]/2)); 
			// Negative cost for slower vehicle behind
      costs[i] -= 1 / (back_vehicle[1]/2); 
    }
    // Simple moving average of costs over the last ten iterations
    avg_costs[i] = (avg_costs[i] * 9) + costs[i];
    avg_costs[i] /= 10;
  }
  // Evaluate potential lane change based on lowest cost
  if (lane == 0) {
    return min_element(avg_costs.begin(), avg_costs.end() - 1) - avg_costs.begin();
  } else if (lane == 1) {
    return min_element(avg_costs.begin(), avg_costs.end())  - avg_costs.begin();
  } else {
    return min_element(avg_costs.begin() + 1, avg_costs.end())  - avg_costs.begin();
  }
}
