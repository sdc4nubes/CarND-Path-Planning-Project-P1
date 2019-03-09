#ifndef vehicle_hpp
#define vehicle_hpp

#include <vector>
#include <string>

using namespace std;

class VehiclePlanner {
  public:
    int curr_lane;
		double speed_limit = 22.3;
    double curr_lead_vehicle_speed = speed_limit;
    double target_vehicle_speed;
		int safe_distance = 30;
    // Decides whether to go left, right, or stay in the same lane
    // Returns amount of meters left or right to move
    int lanePlanner(double s, double d, vector<vector<double>> sensor_fusion);
    // Calculates if d value corresponds to left, right, or center lane
    int laneCalc(double d);
    // Calculates the closest vehicle either in front or behind the car
    // Returns distance and speed of that vehicle
    vector<double> closestVehicle(
			double s, int lane, vector<vector<double>> sensor_fusion, bool direction);
    // Assigns costs to each lane on factors such as distance to nearest vehicle & speed
    // Returns the lane with the lowest cost (0 left, 1 middle, 2 right)
    int laneCost(double s, int lane, vector<vector<double>> sensor_fusion);
};
#endif /* vehicle_hpp */
