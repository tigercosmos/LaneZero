#include "Vehicle.h"
#include "Map.h"

Vehicle::Vehicle(int id, VehicleType type, double position, double velocity,
                 int lane_id, double length, double width)
    : id(id), type(type), position_s_m(position), velocity_mps(velocity),
      acceleration_mps2(0.0), current_lane_id(lane_id), length_m(length), width_m(width) {}

void Vehicle::update_kinematics(double delta_t) {
    position_s_m = position_s_m + velocity_mps * delta_t + 0.5 * acceleration_mps2 * delta_t * delta_t;
    velocity_mps = velocity_mps + acceleration_mps2 * delta_t;
}

void Vehicle::calculate_control(const Map& map, const std::vector<Vehicle*>& surrounding_vehicles) {
    // Placeholder for Phase 2 logic
}
