#include "vehicle.hpp"

int VehiclePlanner::lanePlanner(double s, double d, vector<vector<double>> sensor_fusion) {
  int lane = laneCalc(d);
  int new_lane;
  vector <double> vehicle = closestVehicle(s, lane, sensor_fusion, true);
	// Stay in current lane until deciding to change
  curr_lane = lane; 
	// if adequate space in front and not in right lane,
	// stay in lane and go near the speed limit
  if (vehicle[0] > unsafe_distance && lane == 1) {
    new_lane = lane;
    target_vehicle_speed = speed_limit_mps;
    return 0;
  } else {
		// Determine new lane based on cost model
    new_lane = laneCost(s, lane, sensor_fusion);
    vehicle = closestVehicle(s, new_lane, sensor_fusion, true);
		target_vehicle_speed = speed_limit_mps;
    if( vehicle[0] < unsafe_distance) target_vehicle_speed = vehicle[1] *.98;
		if (vehicle[0] < unsafe_distance * .5) target_vehicle_speed *= .1;
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
  double velocity = speed_limit_mps; 
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
        if (vehicle_s > s + 10 and (vehicle_s - s + 10) < dist) {
          dist = vehicle_s - s + 10;
          velocity = vehicle_v;
        }
      } else {
				// Capture distance and speed of vehicle directly behind
        if (s - 10 >= vehicle_s and (s - 10 - vehicle_s) < dist) {
          dist = s - vehicle_s - 10;
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
	vector <double> vehicle = closestVehicle(s, lane, sensor_fusion, true);
	double check_speed;
	if (vehicle[0] > unsafe_distance) check_speed = speed_limit_mph;
	else check_speed = vehicle[1];
  for (int i = 0; i < 3; i++) {
		// Lane Cost
    costs[i] = fabs(i % 2 -1) * 4;
		if (i == 0) costs[i] -= .5;
		// Get closest vehicle ahead and behind distance and speed for each lane
    front_vehicle = closestVehicle(s, i, sensor_fusion, true);
    back_vehicle = closestVehicle(s, i, sensor_fusion, false);
		// Prohibitive cost for vehicle ahead too close
    if (lane != i && front_vehicle[0] < unsafe_distance) costs[i] = 15; 
		// Positive cost for slower vehicle in front
		if ((i == 1 && front_vehicle[0] < unsafe_distance * 1.1) ||
			(i != 1 && front_vehicle[0] < unsafe_distance))
			if (front_vehicle[1] <= check_speed) costs[i] = 15;
		if (front_vehicle[0] < unsafe_distance * 10) {
			if (front_vehicle[1] > check_speed || i == 1) {
				costs[i] += 6 - vehicle[1] / 10;
				if (i == 1 && check_speed <= front_vehicle[1]) costs[i] * 1.5;
			}
			else costs[i] = 15;
		}
		if (lane != i && back_vehicle[0] < unsafe_distance * 2) costs[i] = 15;
  }
	costs[lane] = min(14., costs[lane]);
  // Evaluate potential lane change based on lowest cost
  if (lane == 0)
    return min_element(costs.begin(), costs.end() - 1) - costs.begin();
  else if (lane == 1)
		return min_element(costs.begin(), costs.end())  - costs.begin();
  else return min_element(costs.begin() + 1, costs.end())  - costs.begin();
}
