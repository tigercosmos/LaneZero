#pragma once

#include <vector>

class Map;

enum class VehicleType {
    Car,
    Truck
};

class Vehicle {
public:
    int id;
    VehicleType type;
    double position_s_m;
    double velocity_mps;
    double acceleration_mps2;
    int current_lane_id;
    double length_m;
    double width_m;

    Vehicle(int id, VehicleType type, double position, double velocity, 
            int lane_id, double length, double width);

    void update_kinematics(double delta_t);

    virtual void calculate_control(const Map& map, const std::vector<Vehicle*>& surrounding_vehicles);

    virtual ~Vehicle() = default;
};
