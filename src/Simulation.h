#pragma once

#include "Map.h"
#include "Vehicle.h"
#include <vector>

class Simulation {
public:
    Map simulation_map;
    std::vector<Vehicle*> vehicles;
    double current_time_s;

    Simulation();
    ~Simulation();

    void run(double duration_s, double delta_t_s);
    void spawn_traffic(int num_vehicles);
    bool check_collision();
};
